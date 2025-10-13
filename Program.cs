// ==============================================
// CanMotorControl.cs (v3.1)
// ✅ SocketCAN + logic CANopen chuẩn DS402
// ✅ Giới hạn hành trình + giám sát vị trí
// ✅ Fault Reset, kiểm tra trạng thái, set mode sớm
// ==============================================
using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading;

namespace dieukhienxoaynang;

public class SocketCANInterface
{
    const int AF_CAN = 29, SOCK_RAW = 3, CAN_RAW = 1;
    const uint SIOCGIFINDEX = 0x8933;
    const int IFNAMSIZ = 16;
    const short POLLIN = 0x0001;

    [DllImport("libc")] static extern int socket(int domain, int type, int protocol);
    [DllImport("libc")] static extern int ioctl(int fd, uint request, ref Ifreq ifr);
    [DllImport("libc")] static extern int bind(int sockfd, nint addr, uint addrlen);
    [DllImport("libc")] static extern int write(int fd, nint buf, nuint count);
    [DllImport("libc")] static extern int read(int fd, nint buf, nuint count);
    [DllImport("libc")] static extern int close(int fd);
    [DllImport("libc")] static extern int poll([In, Out] Pollfd[] fds, uint nfds, int timeout);

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    struct Ifreq
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = IFNAMSIZ)]
        public string ifr_name;
        public int ifr_ifindex;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct Sockaddr_can
    {
        public ushort can_family;
        public int can_ifindex;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)] public byte[] rest;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Can_frame
    {
        public uint can_id;
        public byte can_dlc;
        public byte __pad, __res0, __res1;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public byte[] data;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct Pollfd
    {
        public int fd;
        public short events;
        public short revents;
    }

    public int Sock { get; private set; } = -1;

    private static string ExecuteCommand(string command)
    {
        try
        {
            var p = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-c \"{command}\"",
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                }
            };
            p.Start();
            string output = p.StandardOutput.ReadToEnd();
            string error = p.StandardError.ReadToEnd();
            p.WaitForExit();
            return string.IsNullOrEmpty(error) || error.Contains("RTNETLINK") ? output : error;
        }
        catch (Exception ex) { return ex.Message; }
    }

    public bool Open(string ifname, int baudrate)
    {
        try
        {
            Console.WriteLine($"🔧 Thiết lập CAN {ifname} @ {baudrate}bps...");
            ExecuteCommand($"sudo ip link set {ifname} down");
            ExecuteCommand($"sudo ip link set {ifname} type can bitrate {baudrate}");
            ExecuteCommand($"sudo ip link set {ifname} up");

            string s = ExecuteCommand($"ip -details link show {ifname}");
            if (!s.Contains("UP") || !s.Contains("can"))
            {
                Console.WriteLine($"[❌] Không thể bật {ifname}");
                return false;
            }

            Sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (Sock < 0) return false;

            Ifreq ifr = new() { ifr_name = ifname };
            if (ioctl(Sock, SIOCGIFINDEX, ref ifr) < 0) return false;

            Sockaddr_can addr = new()
            {
                can_family = AF_CAN,
                can_ifindex = ifr.ifr_ifindex,
                rest = new byte[8]
            };
            nint pAddr = Marshal.AllocHGlobal(Marshal.SizeOf(addr));
            Marshal.StructureToPtr(addr, pAddr, false);
            _ = bind(Sock, pAddr, (uint)Marshal.SizeOf(addr));
            Marshal.FreeHGlobal(pAddr);

            Console.WriteLine($"[OK] {ifname} đã sẵn sàng.");
            return true;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[ERROR] {ex.Message}");
            return false;
        }
    }

    public void Close() { if (Sock >= 0) _ = close(Sock); }

    public void Send(Can_frame f)
    {
        int sz = Marshal.SizeOf(f);
        nint p = Marshal.AllocHGlobal(sz);
        Marshal.StructureToPtr(f, p, false);
        _ = write(Sock, p, (nuint)sz);
        Marshal.FreeHGlobal(p);
        Thread.Sleep(5);
    }

    public bool Read(out Can_frame frame, int timeout_ms)
    {
        frame = new Can_frame { data = new byte[8] };
        Pollfd[] fds = new Pollfd[1];
        fds[0].fd = Sock;
        fds[0].events = POLLIN;
        int ret = poll(fds, 1, timeout_ms);
        if (ret > 0 && (fds[0].revents & POLLIN) != 0)
        {
            int size = Marshal.SizeOf(typeof(Can_frame));
            nint p = Marshal.AllocHGlobal(size);
            int r = read(Sock, p, (nuint)size);
            if (r > 0)
                frame = (Can_frame)Marshal.PtrToStructure(p, typeof(Can_frame))!;
            Marshal.FreeHGlobal(p);
            return true;
        }
        return false;
    }

    public void Flush()
    {
        Pollfd[] fds = new Pollfd[1];
        fds[0].fd = Sock; fds[0].events = POLLIN;
        while (poll(fds, 1, 0) > 0)
        {
            int sz = Marshal.SizeOf(typeof(Can_frame));
            nint p = Marshal.AllocHGlobal(sz);
            _ = read(Sock, p, (nuint)sz);
            Marshal.FreeHGlobal(p);
        }
    }
}

class CanMotorControl
{
    const uint SDO_CLI_BASE = 0x600;
    const uint SDO_SRV_BASE = 0x580;
    static readonly SocketCANInterface can = new();

    // --- SDO helpers ---
    static bool WaitSdoResp(byte nodeId, ushort index, byte sub, int timeout_ms)
    {
        int waited = 0;
        while (waited < timeout_ms)
        {
            if (can.Read(out var f, 50))
            {
                if (f.can_id == SDO_SRV_BASE + nodeId &&
                    f.data[1] == (byte)(index & 0xFF) &&
                    f.data[2] == (byte)(index >> 8) &&
                    f.data[3] == sub)
                    return true;
            }
            waited += 50;
        }
        return false;
    }

    static bool SdoWrite(byte nodeId, ushort index, byte sub, uint val, byte len, int timeout_ms = 500)
    {
        can.Flush();
        byte cs = len switch { 1 => 0x2F, 2 => 0x2B, _ => 0x23 };
        var f = new SocketCANInterface.Can_frame
        {
            can_id = SDO_CLI_BASE + nodeId,
            can_dlc = 8,
            data = new byte[8]
        };
        f.data[0] = cs;
        f.data[1] = (byte)(index & 0xFF);
        f.data[2] = (byte)(index >> 8);
        f.data[3] = sub;
        for (int i = 0; i < len; i++) f.data[4 + i] = (byte)(val >> (8 * i));
        can.Send(f);
        if (!WaitSdoResp(nodeId, index, sub, timeout_ms))
        {
            Console.WriteLine($"⏱️ SDO timeout 0x{index:X4}");
            return false;
        }
        return true;
    }

    static bool SdoRead(byte nodeId, ushort index, byte sub, byte[] buf)
    {
        can.Flush();
        var f = new SocketCANInterface.Can_frame
        {
            can_id = SDO_CLI_BASE + nodeId,
            can_dlc = 8,
            data = new byte[8] { 0x40, (byte)(index & 0xFF), (byte)(index >> 8), sub, 0, 0, 0, 0 }
        };
        can.Send(f);
        if (!WaitSdoResp(nodeId, index, sub, 500)) return false;
        can.Read(out var resp, 50);
        for (int i = 0; i < 4; i++) buf[i] = resp.data[4 + i];
        return true;
    }

    static ushort ReadStatus(byte nodeId)
    {
        byte[] b = new byte[4];
        if (!SdoRead(nodeId, 0x6041, 0x00, b)) return 0;
        return (ushort)(b[0] | (b[1] << 8));
    }

    static int ReadPos(byte nodeId)
    {
        byte[] b = new byte[4];
        if (!SdoRead(nodeId, 0x6063, 0x00, b)) return int.MinValue;
        return BitConverter.ToInt32(b, 0);
    }

    static void WriteCW(byte nodeId, ushort val) => SdoWrite(nodeId, 0x6040, 0x00, val, 2);

    // --- GoOperational chuẩn DS402 ---
    static void GoOperational(byte nodeId)
    {
        var nmt = new SocketCANInterface.Can_frame { can_id = 0x000, can_dlc = 2, data = [0x01, nodeId, 0, 0, 0, 0, 0, 0] };
        can.Send(nmt);
        Thread.Sleep(200);

        SdoWrite(nodeId, 0x6060, 0x00, 1, 1); // chọn Profile Position sớm
        Thread.Sleep(100);

        WriteCW(nodeId, 0x0080); // Reset Fault
        Thread.Sleep(200);
        WriteCW(nodeId, 0x0006); // Ready
        Thread.Sleep(200);
        WriteCW(nodeId, 0x0007); // Switched On
        Thread.Sleep(200);
        WriteCW(nodeId, 0x000F); // Operation Enabled
        Thread.Sleep(300);

        ushort st = ReadStatus(nodeId);
        Console.WriteLine($"Status after enable: 0x{st:X4}");
    }

    static void StopMotor(byte nodeId)
    {
        WriteCW(nodeId, 0x000B);
        Thread.Sleep(100);
        WriteCW(nodeId, 0x0000);
        Console.WriteLine("\n⚠️ Motor stopped (limit exceeded).");
    }

    static bool MoveTo(byte nodeId, int target, int velDEC)
    {
        if (!SdoWrite(nodeId, 0x6081, 0x00, (uint)velDEC, 4)) return false;
        if (!SdoWrite(nodeId, 0x607A, 0x00, (uint)target, 4)) return false;
        WriteCW(nodeId, 0x001F); Thread.Sleep(50);
        WriteCW(nodeId, 0x000F);
        return true;
    }

    static bool WaitReached(byte nodeId, int target, int tol = 500, int timeout_s = 30)
    {
        int stable = 0;
        for (int i = 0; i < timeout_s * 10; i++)
        {
            int pos = ReadPos(nodeId);
            if (pos == int.MinValue) continue;
            int diff = Math.Abs(pos - target);
            Console.Write($"\rpos={pos,8} diff={diff,6}");
            if (diff <= tol)
            {
                if (++stable > 5) { Console.WriteLine("\n✅ Reached."); return true; }
            }
            else stable = 0;
            Thread.Sleep(100);
        }
        Console.WriteLine("\n⚠️ Timeout waiting for position.");
        return false;
    }

    static void Main()
    {
        string ifname = "can0";
        byte nodeId = 2;
        int posA = 0, posB = 3_700_000, vel = 5_000_000;

        if (!can.Open(ifname, 50000)) return;
        Console.WriteLine($"Connected to {ifname}");

        GoOperational(nodeId);

        Console.WriteLine("⬆️  UP: TOP\n⬇️  DOWN: BOTTOM\nESC: Exit");

        bool run = true;
        while (run)
        {
            int pos = ReadPos(nodeId);
            if (pos != int.MinValue)
            {
                Console.Write($"\rCurrent: {pos,8}");
                if (pos < posA - 5000 || pos > posB + 5000)
                {
                    StopMotor(nodeId);
                    break;
                }
            }

            if (Console.KeyAvailable)
            {
                var k = Console.ReadKey(true).Key;
                if (k == ConsoleKey.Escape) break;
                if (k == ConsoleKey.UpArrow)
                {
                    if (pos > posB - 10000)
                    {
                        Console.WriteLine("\n⚠️ Near top limit!");
                        continue;
                    }
                    Console.WriteLine($"\n>>> Move to TOP ({posB})");
                    if (MoveTo(nodeId, posB, vel)) WaitReached(nodeId, posB);
                }
                else if (k == ConsoleKey.DownArrow)
                {
                    if (pos < posA + 10000)
                    {
                        Console.WriteLine("\n⚠️ Near bottom limit!");
                        continue;
                    }
                    Console.WriteLine($"\n<<< Move to BOTTOM ({posA})");
                    if (MoveTo(nodeId, posA, vel)) WaitReached(nodeId, posA);
                }
            }
            Thread.Sleep(100);
        }

        can.Close();
        Console.WriteLine("\nSocket closed.");
    }
}
