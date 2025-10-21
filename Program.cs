using System;
using System.Diagnostics;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Xml.Linq;

namespace dieukhienxoaynang
{
    public static class UbuntuCanInterface
    {
        private static bool canConfigured = false;

        private static void ConfigureCanInterface(string interfaceName, int baudrate)
        {
            if (canConfigured) return;
            canConfigured = true;

            Console.WriteLine($"[CAN Setup] Thiết lập {interfaceName} ({baudrate})");
            ExecuteCommand($"sudo ip link set {interfaceName} down");
            ExecuteCommand($"sudo ip link set {interfaceName} type can bitrate {baudrate}");
            ExecuteCommand($"sudo ip link set {interfaceName} up");
        }

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
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    }
                };
                p.Start();
                string output = p.StandardOutput.ReadToEnd();
                string error = p.StandardError.ReadToEnd();
                p.WaitForExit();
                return output + error;
            }
            catch (Exception ex)
            {
                return $"❌ {ex.Message}";
            }
        }

        public static int StartCanOpen(string interfaceName, int baudrate)
        {
            ConfigureCanInterface(interfaceName, baudrate);

            const int AF_CAN = 29;
            const int SOCK_RAW = 3;
            const int CAN_RAW = 1;

            int sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (sock < 0) return -1;

            int ifIndex = if_nametoindex(interfaceName);
            if (ifIndex == 0) return -1;

            var addr = new sockaddr_can { can_family = AF_CAN, can_ifindex = ifIndex };
            if (bind(sock, ref addr, Marshal.SizeOf(addr)) < 0)
            {
                close(sock);
                return -1;
            }

            Console.WriteLine($"✅ Kết nối {interfaceName} thành công (socket={sock})");
            return sock;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct sockaddr_can
        {
            public ushort can_family;
            public int can_ifindex;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] can_addr;
        }

        [DllImport("libc")] private static extern int socket(int domain, int type, int protocol);
        [DllImport("libc")] private static extern int bind(int sockfd, ref sockaddr_can addr, int addrlen);
        [DllImport("libc")] private static extern int close(int fd);
        [DllImport("libc")] private static extern int if_nametoindex(string ifname);
    }

    public class ControlSpeed
    {
        public volatile bool running = true;
        public volatile int speed = 0;

        public int GetSpeed() => speed;
        public void SetSpeed(int v) => speed = v;
        public void Stop() => running = false;
        public bool IsRunning() => running;
    }

    public class Node1_Rotate
    {
        private readonly uint nodeId = 1;
        private readonly string iface;
        private readonly int baud;
        public readonly ControlSpeed Control = new();
        private readonly double gear = 350;

        public Node1_Rotate(string iface, int baud)
        {
            this.iface = iface; this.baud = baud;
        }

        public void Run()
        {
            int sock = UbuntuCanInterface.StartCanOpen(iface, baud);
            if (sock < 0) return;
            ControlPDO.ConfigurePDO_Default(sock, nodeId);
            Thread.Sleep(200);
            ControlPDO.EnableMotorVelocityMode(sock, nodeId);
            new Thread(() => ControlPDO.ListenTPDO(sock, nodeId, "Rotate", Control.IsRunning)).Start();

            while (Control.IsRunning())
            {
                long spd = (long)(Control.GetSpeed() * gear);
                ControlPDO.SendVelocity(sock, nodeId, (int)spd);
                Thread.Sleep(200);
            }

            ControlPDO.SendVelocity(sock, nodeId, 0);
            Console.WriteLine("[Rotate] stopped.");
        }
    }

    public class Node2_Lift
    {
        private readonly uint nodeId = 2;
        private readonly string iface;
        private readonly int baud;
        public readonly ControlSpeed Control = new();
        private readonly double factor = 33;

        public Node2_Lift(string iface, int baud)
        {
            this.iface = iface; this.baud = baud;
        }

        public void Run()
        {
            int sock = UbuntuCanInterface.StartCanOpen(iface, baud);
            if (sock < 0) return;
            ControlPDO.ConfigurePDO_Default(sock, nodeId);
            Thread.Sleep(200);
            ControlPDO.EnableMotorVelocityMode(sock, nodeId);
            new Thread(() => ControlPDO.ListenTPDO(sock, nodeId, "Lift", Control.IsRunning)).Start();

            while (Control.IsRunning())
            {
                long spd = (long)(Control.GetSpeed() * factor);
                ControlPDO.SendVelocity(sock, nodeId, (int)spd);
                Thread.Sleep(200);
            }

            ControlPDO.SendVelocity(sock, nodeId, 0);
            Console.WriteLine("[Lift] stopped.");
        }
    }

    public class InputManager
    {
        private readonly ControlSpeed rotateCtrl;
        private readonly ControlSpeed liftCtrl;

        public InputManager(ControlSpeed r, ControlSpeed l)
        {
            rotateCtrl = r;
            liftCtrl = l;
        }

        public void Start()
        {
            new Thread(Run) { IsBackground = true }.Start();
        }

        private void Run()
        {
            ConsoleKey key;
            int speedStep = 5;
            while ((key = Console.ReadKey(true).Key) != ConsoleKey.C)
            {
                switch (key)
                {
                    case ConsoleKey.W: rotateCtrl.SetSpeed(rotateCtrl.GetSpeed() + speedStep); break;
                    case ConsoleKey.S: rotateCtrl.SetSpeed(rotateCtrl.GetSpeed() - speedStep); break;
                    case ConsoleKey.UpArrow: liftCtrl.SetSpeed(liftCtrl.GetSpeed() + speedStep); break;
                    case ConsoleKey.DownArrow: liftCtrl.SetSpeed(liftCtrl.GetSpeed() - speedStep); break;
                    case ConsoleKey.X:
                        rotateCtrl.Stop(); liftCtrl.Stop();
                        break;
                }

                Console.WriteLine($"[Rotate={rotateCtrl.GetSpeed()} | Lift={liftCtrl.GetSpeed()}]");
            }

            rotateCtrl.Stop(); liftCtrl.Stop();
        }
    }

    internal class Program
    {
        static void Main()
        {
            Console.Clear();
            Console.WriteLine("==== CANopen Dual Motor Controller ====");
            Console.WriteLine("W/S: Rotate speed | ↑/↓: Lift speed | X: Stop both | C: Exit");

            var node1 = new Node1_Rotate("can0", 500000);
            var node2 = new Node2_Lift("can0", 500000);

            var t1 = new Thread(node1.Run);
            var t2 = new Thread(node2.Run);
            t1.Start();
            t2.Start();

            var input = new InputManager(node1.Control, node2.Control);
            input.Start();

            t1.Join();
            t2.Join();

            Console.WriteLine("✅ Both nodes exited.");
        }
    }
    
    public class ControlPDO
    {
        // Biến lưu giá trị profile velocity trước đó để tránh gửi lặp
        static int preProfileVelocity;

        // Cấu trúc cho frame CAN
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CanFrame
        {
            public uint can_id;     // ID khung CAN (COB-ID)
            public byte can_dlc;    // Độ dài dữ liệu (0–8 byte)
            public byte __pad1, __pad2, __pad3;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] data;     // Dữ liệu thực
        }

        // Import hàm write từ libc
        [DllImport("libc", SetLastError = true)]
        public static extern int write(int fd, ref CanFrame frame, int len);

        // Import hàm read từ libc
        [DllImport("libc", SetLastError = true)]
        public static extern int read(int fd, ref CanFrame frame, int len);

        // Độ phân giải encoder mặc định
        static readonly int encoder_Resolution = 10000;
        // Hàm gửi frame CAN
        public static void SendFrame(int __sock, uint canId, byte[] payload)
        {
            CanFrame frame = new()
            {
                can_id = canId,
                // Gán độ dài dữ liệu từ độ dài mảng payload, giới hạn bằng byte
                can_dlc = (byte)payload.Length
            };
            // Đặt các byte padding (dự phòng) về 0
            frame.__pad1 = frame.__pad2 = frame.__pad3 = 0;
            frame.data = new byte[8];
            // Sao chép dữ liệu từ payload vào mảng data của frame
            Array.Copy(payload, frame.data, payload.Length);
            // Gửi frame qua socket CAN, sử dụng hàm write từ libc
            int nbytes = write(__sock, ref frame, Marshal.SizeOf(typeof(CanFrame)));
            // Kiểm tra nếu gửi thất bại (nbytes < 0), in thông báo lỗi
            if (nbytes < 0)
                Console.WriteLine("Lỗi khi gửi frame!");
        }

        // Ghi giá trị uint32 12345678 (0x00BC614E) vào node 1, index 0x6040, sub-index 0
        //WriteSDO_U32(sock, 1, 0x6040, 0, 12345678);
        public static void WriteSDO_U32(int sock, uint nodeId, ushort index, byte subIndex, uint value)
        {
            // Tính ID truyền SDO dựa trên nodeId
            uint sdoTxId = 0x600 + nodeId;
            // Tạo mảng byte chứa dữ liệu SDO
            byte[] data =
            [
                0x23, // Mã lệnh viết 4 byte
            (byte)(index & 0xFF), // Byte thấp của index
            (byte)(index >> 8), // Byte cao của index
            subIndex, // Sub-index
            (byte)(value & 0xFF), // Byte thấp của value
            (byte)(value >> 8 & 0xFF), // Byte thứ hai của value
            (byte)(value >> 16 & 0xFF), // Byte thứ ba của value
            (byte)(value >> 24 & 0xFF), // Byte cao nhất của value
        ];
            // Gửi frame SDO
            SendFrame(sock, sdoTxId, data);
            // Đợi 10ms để đảm bảo truyền ổn định
            Thread.Sleep(10);
        }

        // Ghi giá trị uint8 5 vào node 1, index 0x6040, sub-index 1
        //WriteSDO_U8(sock, 1, 0x6040, 1, 5);
        public static void WriteSDO_U8(int sock, uint nodeId, ushort index, byte subIndex, byte value)
        {
            // Tính ID truyền SDO dựa trên nodeId
            uint sdoTxId = 0x600 + nodeId;
            // Tạo mảng byte chứa dữ liệu SDO với kích thước 8
            byte[] data = new byte[8];
            data[0] = 0x2F; // Mã lệnh viết 1 byte
            data[1] = (byte)(index & 0xFF); // Byte thấp của index
            data[2] = (byte)(index >> 8); // Byte cao của index
            data[3] = subIndex; // Sub-index
            data[4] = value; // Giá trị byte
                             // Gửi frame SDO
            SendFrame(sock, sdoTxId, data);
            // Đợi 10ms để đảm bảo truyền ổn định
            Thread.Sleep(10);
        }

        // Cấu hình PDO mặc định cho node
        public static void ConfigurePDO_Default(int sock, uint nodeId)
        {
            Console.WriteLine("Configuring PDO mapping...");

            // Disable TPDO trước khi map
            WriteSDO_U32(sock, nodeId, 0x1800, 1, 0x80000180 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1801, 1, 0x80000280 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1802, 1, 0x80000380 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1803, 1, 0x80000480 + nodeId);

            // Map TPDO1: Statusword, Mode, Error
            WriteSDO_U8(sock, nodeId, 0x1A00, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A00, 1, 0x60410010);
            WriteSDO_U32(sock, nodeId, 0x1A00, 2, 0x60610008);
            WriteSDO_U32(sock, nodeId, 0x1A00, 3, 0x603F0010);
            WriteSDO_U8(sock, nodeId, 0x1A00, 0, 3);
            WriteSDO_U32(sock, nodeId, 0x1800, 1, 0x00000180 + nodeId);

            // Map TPDO2: Position actual
            WriteSDO_U8(sock, nodeId, 0x1A01, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A01, 1, 0x60640020);
            WriteSDO_U8(sock, nodeId, 0x1A01, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1801, 1, 0x00000280 + nodeId);

            // Map TPDO3: Velocity actual
            WriteSDO_U8(sock, nodeId, 0x1A02, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A02, 1, 0x606C0020);
            WriteSDO_U8(sock, nodeId, 0x1A02, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1802, 1, 0x00000380 + nodeId);

            // Map TPDO4: Position & Velocity actual
            WriteSDO_U8(sock, nodeId, 0x1A03, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A03, 1, 0x60640020);
            WriteSDO_U32(sock, nodeId, 0x1A03, 2, 0x606C0020);
            WriteSDO_U8(sock, nodeId, 0x1A03, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1803, 1, 0x00000480 + nodeId);

            // Disable RPDO trước khi map
            WriteSDO_U32(sock, nodeId, 0x1400, 1, 0x80000200 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1401, 1, 0x80000300 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1402, 1, 0x80000400 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1403, 1, 0x80000500 + nodeId);

            // Map RPDO1: Controlword
            WriteSDO_U8(sock, nodeId, 0x1600, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1600, 1, 0x60400010);
            WriteSDO_U8(sock, nodeId, 0x1600, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1400, 1, 0x00000200 + nodeId);

            // Map RPDO2: Controlword & Target Position
            WriteSDO_U8(sock, nodeId, 0x1601, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1601, 1, 0x60400010);
            WriteSDO_U32(sock, nodeId, 0x1601, 2, 0x607A0020);
            WriteSDO_U8(sock, nodeId, 0x1601, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1401, 1, 0x00000300 + nodeId);

            // Map RPDO3: Controlword & Target Velocity
            WriteSDO_U8(sock, nodeId, 0x1602, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1602, 1, 0x60400010);
            WriteSDO_U32(sock, nodeId, 0x1602, 2, 0x60FF0020);
            WriteSDO_U8(sock, nodeId, 0x1602, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1402, 1, 0x00000400 + nodeId);

            // Map RPDO4: Digital Output
            WriteSDO_U8(sock, nodeId, 0x1603, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1603, 1, 0x60FE0120);
            WriteSDO_U8(sock, nodeId, 0x1603, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1403, 1, 0x00000500 + nodeId);

            Console.WriteLine("PDO mapping configured to default.");
        }

        // Gửi controlword qua RPDO1
        public static void SendControlword(int __sock, uint nodeId, ushort controlword)
        {
            uint rpdo1Id = 0x200 + nodeId;
            byte[] data = new byte[8];
            data[0] = (byte)(controlword & 0xFF);
            data[1] = (byte)(controlword >> 8);
            SendFrame(__sock, rpdo1Id, data);
            Console.WriteLine($"[TPDO1] Controlword = 0x{controlword:X4}");
        }
        // Đọc statusword qua SDO
        public static ushort ReadStatusword(int __sock, uint nodeId)
        {
            uint sdoTxId = 0x600 + nodeId;
            uint sdoRxId = 0x580 + nodeId;
            byte[] req = [0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0];
            SendFrame(__sock, sdoTxId, req);

            CanFrame rxFrame = new();
            int n = read(__sock, ref rxFrame, Marshal.SizeOf(typeof(CanFrame)));
            if (n > 0 && rxFrame.can_id == sdoRxId)
            {
                return BitConverter.ToUInt16(rxFrame.data, 4);
            }
            return 0;
        }
        // Gửi vị trí target qua RPDO2, cập nhật profile velocity nếu thay đổi
        public static void SendPosition(int __sock, uint nodeId, int targetPos, int profileVelocity)
        {
            uint rpdo2Id = 0x300 + nodeId;
            uint sdoTxId = 0x600 + nodeId;

            // Cập nhật profile velocity nếu khác trước
            if (profileVelocity != preProfileVelocity)
            {
                byte[] velData = new byte[8];
                velData[0] = 0x23;
                velData[1] = 0x81;
                velData[2] = 0x60;
                velData[3] = 0x00;
                long _profile_velocity = (long)profileVelocity * 512 * encoder_Resolution / 1875;
                byte[] vbytes = BitConverter.GetBytes(_profile_velocity);
                Array.Copy(vbytes, 0, velData, 4, 4);
                for (int i = 0; i < 3; i++)
                {
                    SendFrame(__sock, sdoTxId, velData);
                }
                preProfileVelocity = profileVelocity;
                Thread.Sleep(50);
            }

            // Gửi setpoint với new setpoint bit
            ushort baseCw = 0x000F;
            byte[] posBytes = BitConverter.GetBytes(targetPos);

            ushort cw_setpoint = (ushort)(baseCw | 1 << 4 | 1 << 5);
            byte[] data = new byte[8];
            data[0] = (byte)(cw_setpoint & 0xFF);
            data[1] = (byte)(cw_setpoint >> 8);
            Array.Copy(posBytes, 0, data, 2, 4);
            SendFrame(__sock, rpdo2Id, data);

            Thread.Sleep(10);
            // Clear new setpoint bit
            ushort cw_clear = (ushort)(baseCw | 1 << 5);
            byte[] data2 = new byte[8];
            data2[0] = (byte)(cw_clear & 0xFF);
            data2[1] = (byte)(cw_clear >> 8);
            Array.Copy(posBytes, 0, data2, 2, 4);
            SendFrame(__sock, rpdo2Id, data2);

            Console.WriteLine($"[RPDO2] TargetPos={targetPos}, ProfileVel={profileVelocity}");
        }

        // Gửi velocity target qua RPDO3
        public static void SendVelocity(int __sock, uint nodeId, int velocity, ushort controlword = 0x000F)
        {
            uint rpdo3Id = 0x400 + nodeId;
            byte[] data = new byte[6];

            data[0] = (byte)(controlword & 0xFF);
            data[1] = (byte)(controlword >> 8);
            long _velocity = (long)velocity * 512 * encoder_Resolution / 1875;
            byte[] velBytes = BitConverter.GetBytes(_velocity);
            Array.Copy(velBytes, 0, data, 2, 4);

            SendFrame(__sock, rpdo3Id, data);
            Console.WriteLine($"[TPDO3] Velocity target = {_velocity}");
        }

        // Lắng nghe TPDO từ node
        public static void ListenTPDO(int sock, uint nodeId, string label, Func<bool> runningCheck)
        {
            while (runningCheck())
            {
                CanFrame rx = new();
                int nbytes = read(sock, ref rx, Marshal.SizeOf(typeof(CanFrame)));
                if (nbytes > 0)
                {
                    // Xử lý TPDO1: Statusword
                    if (rx.can_id == 0x180 + nodeId)
                    {
                        ushort status = BitConverter.ToUInt16(rx.data, 0);
                        Console.WriteLine($"[RPDO1] Statusword = 0x{status:X4}");
                    }
                    // Xử lý TPDO2: Position actual
                    if (rx.can_id == 0x280 + nodeId)
                    {
                        int pos = BitConverter.ToInt32(rx.data, 0);
                        Console.WriteLine($"[RPDO2: Positon actual = {pos}");
                    }
                    // Xử lý TPDO3: Velocity actual
                    if (rx.can_id == 0x380 + nodeId)
                    {
                        long _vel = BitConverter.ToInt32(rx.data, 0);
                        long vel = _vel * 1875 / (512 * encoder_Resolution * 350);
                        Console.WriteLine($"[RPDO3] Velocity actual = {vel}");
                    }
                    // Xử lý TPDO4: Position & Velocity actual
                    if (rx.can_id == 0x480 + nodeId)
                    {
                        long _pos = BitConverter.ToInt32(rx.data, 0);
                        long _vel = BitConverter.ToInt32(rx.data, 4);
                        Console.WriteLine($"[RPDO4] Position actual = {_pos}");
                        Console.WriteLine($"[RPDO4] Velocity actual = {_vel}");
                    }
                }
                Thread.Sleep(5);
            }
        }

        // Kích hoạt mode velocity cho motor
        public static void EnableMotorVelocityMode(int ___sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + _nodeId;
            // Set mode velocity (3)
            SendFrame(___sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Read mode
            SendFrame(___sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            // Shutdown
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Switch on
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Enable operation
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            // Set target velocity 0
            SendFrame(___sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
        }

        // Kích hoạt mode position cho motor
        public static void EnableMotorPositionMode(int ___sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + _nodeId;
            // Set mode position (1)
            SendFrame(___sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Read mode
            SendFrame(___sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            // Shutdown
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Switch on
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Enable operation
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            // Set target velocity 0
            SendFrame(___sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
        }

        // Kích hoạt mode homing cho motor
        public static void EnableMotorHomeMode(int ___sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + _nodeId;
            // Set mode homing (6)
            SendFrame(___sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Read mode
            SendFrame(___sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            // Shutdown
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Switch on
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            // Enable operation
            SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            // Set target velocity 0
            SendFrame(___sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
        }
        // Đặt giới hạn vị trí phần mềm
    }
}
