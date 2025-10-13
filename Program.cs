// ==============================================
// CanMotorControl.cs (v3) 
// ✅ SocketCAN + giữ nguyên logic CANopen
// ✅ Giới hạn hành trình + giám sát vị trí
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
            var process = new Process
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
            process.Start();
            string output = process.StandardOutput.ReadToEnd();
            string error = process.StandardError.ReadToEnd();
            process.WaitForExit();
            if (!string.IsNullOrEmpty(error) && !error.Contains("RTNETLINK"))
                return error;
            return output;
        }
        catch (Exception ex) { return $"Lỗi: {ex.Message}"; }
    }
    public bool Open(string interfaceName, int baudrate)
    {
        try
        {
            // --- 1. Thiết lập giao diện CAN ---
            Console.WriteLine($"🔧 Thiết lập CAN interface {interfaceName} với baudrate {baudrate}...");

            ExecuteCommand($"sudo ip link set {interfaceName} down");
            ExecuteCommand($"sudo ip link set {interfaceName} type can bitrate {baudrate}");
            ExecuteCommand($"sudo ip link set {interfaceName} up");

            string status = ExecuteCommand($"ip -details link show {interfaceName}");
            if (!status.Contains("UP") || !status.Contains("can"))
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"[LỖI] Không thể bật giao diện {interfaceName}!");
                Console.ResetColor();
                return false;
            }

            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine($"[OK] Giao diện {interfaceName} đã sẵn sàng.");
            Console.ResetColor();

            // --- 2. Tạo socket raw CAN ---
            Sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (Sock < 0)
            {
                Console.Error.WriteLine("socket() failed");
                return false;
            }

            Ifreq ifr = new() { ifr_name = interfaceName };
            if (ioctl(Sock, SIOCGIFINDEX, ref ifr) < 0)
            {
                Console.Error.WriteLine("ioctl() failed");
                _ = close(Sock);
                return false;
            }

            Sockaddr_can addr = new()
            {
                can_family = AF_CAN,
                can_ifindex = ifr.ifr_ifindex,
                rest = new byte[8]
            };

            nint pAddr = Marshal.AllocHGlobal(Marshal.SizeOf(addr));
            Marshal.StructureToPtr(addr, pAddr, false);

            if (bind(Sock, pAddr, (uint)Marshal.SizeOf(addr)) < 0)
            {
                Console.Error.WriteLine("bind() failed");
                Marshal.FreeHGlobal(pAddr);
                _ = close(Sock);
                return false;
            }

            Marshal.FreeHGlobal(pAddr);
            Thread.Sleep(200);
            return true;
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"[ERROR] Lỗi khởi tạo CAN: {ex.Message}");
            Console.ResetColor();
            return false;
        }
    }

    public void Close() { if (Sock >= 0) _ = close(Sock); }

    public void Send(Can_frame f)
    {
        int size = Marshal.SizeOf(f);
        nint p = Marshal.AllocHGlobal(size);
        Marshal.StructureToPtr(f, p, false);
        int written = write(Sock, p, (nuint)size);
        if (written != size)
            Console.Error.WriteLine($"write() partial {written}/{size}");
        Marshal.FreeHGlobal(p);
        Thread.Sleep(8);
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
            {
                object? obj = Marshal.PtrToStructure(p, typeof(Can_frame));
                if (obj is Can_frame cf)
                    frame = cf;
                else
                    frame = new Can_frame { data = new byte[8] };
                Marshal.FreeHGlobal(p);
                return true;
            }
            Marshal.FreeHGlobal(p);
        }
        return false;
    }

    public void Flush()
    {
        _ = new Can_frame() { data = new byte[8] };
        Pollfd[] fds = new Pollfd[1];
        fds[0].fd = Sock;
        fds[0].events = POLLIN;

        while (true)
        {
            int ret = poll(fds, 1, 0); // timeout=0 => non-blocking
            if (ret > 0 && (fds[0].revents & POLLIN) != 0)
            {
                int size = Marshal.SizeOf(typeof(Can_frame));
                nint p = Marshal.AllocHGlobal(size);
                int r = read(Sock, p, (nuint)size);
                Marshal.FreeHGlobal(p);
                if (r <= 0) break;
            }
            else break;
        }
    }

}

class CanMotorControl
{
    const uint SDO_CLI_BASE = 0x600;
    const uint SDO_SRV_BASE = 0x580;
    static readonly SocketCANInterface can = new();

    static bool WaitForSdoResponse(byte nodeId, ushort index, byte sub, out SocketCANInterface.Can_frame resp, int timeout_ms)
    {
        int waited = 0;
        const int step = 50;
        while (waited < timeout_ms)
        {
            if (can.Read(out resp, step))
            {
                uint expected = SDO_SRV_BASE + nodeId;
                if (resp.can_id == expected &&
                    resp.data[1] == (byte)(index & 0xFF) &&
                    resp.data[2] == (byte)(index >> 8 & 0xFF) &&
                    resp.data[3] == sub)
                    return true;
            }
            waited += step;
        }
        resp = new SocketCANInterface.Can_frame { data = new byte[8] };
        return false;
    }

    static bool SendSdoWriteWait(byte nodeId, ushort index, byte sub, uint data, byte len, int timeout_ms = 500)
    {
        can.Flush(); // 💡 Thêm dòng này
        byte cs = len switch { 1 => 0x2F, 2 => 0x2B, _ => 0x23 };
        var f = new SocketCANInterface.Can_frame
        {
            can_id = SDO_CLI_BASE + nodeId,
            can_dlc = 8,
            data = new byte[8]
        };
        f.data[0] = cs;
        f.data[1] = (byte)(index & 0xFF);
        f.data[2] = (byte)(index >> 8 & 0xFF);
        f.data[3] = sub;
        for (int i = 0; i < len; i++) f.data[4 + i] = (byte)(data >> 8 * i & 0xFF);
        can.Send(f);

        if (!WaitForSdoResponse(nodeId, index, sub, out _, timeout_ms))
        {
            Console.Error.WriteLine($"SDO write timeout 0x{index:X4}");
            return false;
        }
        return true;
    }

    static bool SendSdoRead(byte nodeId, ushort index, byte sub, byte[] outBuf, int timeout_ms = 500)
    {
        can.Flush(); // 💡 Thêm dòng này
        var f = new SocketCANInterface.Can_frame
        {
            can_id = SDO_CLI_BASE + nodeId,
            can_dlc = 8,
            data = new byte[8]
        };
        f.data[0] = 0x40;
        f.data[1] = (byte)(index & 0xFF);
        f.data[2] = (byte)(index >> 8 & 0xFF);
        f.data[3] = sub;
        can.Send(f);
        if (!WaitForSdoResponse(nodeId, index, sub, out var resp, timeout_ms)) return false;
        for (int i = 0; i < 4; i++) outBuf[i] = resp.data[4 + i];
        return true;
    }

    static int ReadActualPosition(byte nodeId)
    {
        byte[] buf = new byte[4];
        if (!SendSdoRead(nodeId, 0x6063, 0x00, buf, 500)) return int.MinValue;
        return BitConverter.ToInt32(buf, 0);
    }

    static ushort ReadStatusword(byte nodeId)
    {
        byte[] buf = new byte[4];
        if (!SendSdoRead(nodeId, 0x6041, 0x00, buf, 400)) return 0;
        return (ushort)(buf[0] | buf[1] << 8);
    }

    static bool WriteControlword(byte nodeId, ushort cw)
        => SendSdoWriteWait(nodeId, 0x6040, 0x00, cw, 2, 500);

    static void StopMotor(byte nodeId)
    {
        WriteControlword(nodeId, 0x000B); // disable voltage
        Thread.Sleep(100);
        WriteControlword(nodeId, 0x0000);
        Console.ForegroundColor = ConsoleColor.Red;
        Console.WriteLine("⚠️ Motor stopped due to position limit exceeded!");
        Console.ResetColor();
    }

    static void GoOperational(byte nodeId)
    {
        var nmt = new SocketCANInterface.Can_frame
        {
            can_id = 0x000,
            can_dlc = 2,
            data = new byte[8]
        };
        nmt.data[0] = 0x01;
        nmt.data[1] = nodeId;
        can.Send(nmt);
        Thread.Sleep(200);

        WriteControlword(nodeId, 0x0006);
        Thread.Sleep(200);
        WriteControlword(nodeId, 0x0007);
        Thread.Sleep(200);
        WriteControlword(nodeId, 0x000F);
        Thread.Sleep(200);

        ushort st = ReadStatusword(nodeId);
        Console.WriteLine($"Status after op enable: 0x{st:X4}");
    }

    static bool SetProfileParamsAndTarget(byte nodeId, int targetPos, int velDEC, int accelDEC = 20000, int decelDEC = 20000)
    {
        if (!SendSdoWriteWait(nodeId, 0x6083, 0x00, (uint)accelDEC, 4)) return false;
        if (!SendSdoWriteWait(nodeId, 0x6084, 0x00, (uint)decelDEC, 4)) return false;
        Thread.Sleep(2);
        if (!SendSdoWriteWait(nodeId, 0x6081, 0x00, (uint)velDEC, 4)) return false;
        Thread.Sleep(2);
        if (!SendSdoWriteWait(nodeId, 0x607A, 0x00, (uint)targetPos, 4)) return false;

        WriteControlword(nodeId, 0x000F);
        Thread.Sleep(100);
        WriteControlword(nodeId, 0x001F);
        Thread.Sleep(100);
        WriteControlword(nodeId, 0x003F);
        Thread.Sleep(50);
        WriteControlword(nodeId, 0x000F);
        return true;
    }

    static bool WaitReachedPositionByPos(byte nodeId, int target, int posTol = 500, int stableCount = 5, int timeout_s = 30)
    {
        int initialPos = ReadActualPosition(nodeId);
        if (initialPos == int.MinValue) return false;
        int lastPos = initialPos;
        bool startedMoving = false;
        int stable = 0;
        int loops = timeout_s * 10;

        for (int i = 0; i < loops; i++)
        {
            int pos = ReadActualPosition(nodeId);
            if (pos == int.MinValue)
            {
                Thread.Sleep(100);
                continue;
            }

            int diff = Math.Abs(pos - target);
            int delta = Math.Abs(pos - lastPos);
            Console.Write($"\rpos={pos,8} diff={diff,6} Δ={delta,6}");

            if (!startedMoving && Math.Abs(pos - initialPos) > 2000)
            {
                startedMoving = true;
                Console.WriteLine($"\nMotor started moving (initialPos={initialPos})");
            }

            if (startedMoving)
            {
                if (diff <= posTol)
                {
                    stable++;
                    if (stable >= stableCount)
                    {
                        Console.WriteLine("\n✅ Reached target position.");
                        return true;
                    }
                }
                else
                {
                    stable = 0;
                }
            }

            lastPos = pos;
            Thread.Sleep(100);
        }

        Console.WriteLine("\n⚠️ Timeout waiting for position reached!");
        return false;
    }

    // ========================= MAIN =========================
    static void Main(string[] args)
    {
        ArgumentNullException.ThrowIfNull(args);
        string ifname = "can0";
        byte nodeId = 1;
        int posA = 0, posB = 3_700_000;
        int velDEC = 5_000_000;

        if (!can.Open(ifname, 50000))
        {
            Console.Error.WriteLine($"Cannot open {ifname}. Run as root!");
            return;
        }

        Console.WriteLine($"Connected to {ifname}");
        GoOperational(nodeId);
        SendSdoWriteWait(nodeId, 0x6060, 0x00, 1, 1, 500);
        Thread.Sleep(100);

        Console.WriteLine("⬆️  UP: Move to TOP\n⬇️  DOWN: Move to BOTTOM\nESC: Exit");

        bool run = true;
        while (run)
        {
            int pos = ReadActualPosition(nodeId);
            if (pos != int.MinValue)
            {
                Console.Write($"\rCurrent Position: {pos,8} ");
                if (pos < posA - 5000 || pos > posB + 5000)
                {
                    StopMotor(nodeId);
                    run = false;
                    break;
                }
            }

            if (Console.KeyAvailable)
            {
                var key = Console.ReadKey(true).Key;
                if (key == ConsoleKey.Escape) break;
                else if (key == ConsoleKey.UpArrow)
                {
                    Console.WriteLine($"\n>>> Moving to TOP ({posB})");
                    if (SetProfileParamsAndTarget(nodeId, posB, velDEC))
                        Console.WriteLine("Moving UP...");
                }
                else if (key == ConsoleKey.DownArrow)
                {
                    Console.WriteLine($"\n<<< Moving to BOTTOM ({posA})");
                    if (SetProfileParamsAndTarget(nodeId, posA, velDEC))
                        Console.WriteLine("Moving DOWN...");
                }
            }

            Thread.Sleep(100);
        }

        can.Close();
        Console.WriteLine("\nSocket closed.");
    }
}