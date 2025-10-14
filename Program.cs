using dieukhienxoaynang;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics.Arm;
using System.Text;
using System.Xml.Linq;

namespace dieukhienxoaynang
{
    public class ControlSpeed
    {
        public static int _speedRotate = 0;
        public static int _speedLift = 0;
        public static bool _running = true;

        [StructLayout(LayoutKind.Sequential)]
        private struct Termios
        {
            public uint c_iflag;
            public uint c_oflag;
            public uint c_cflag;
            public uint c_lflag;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public byte[] c_cc;
            public uint c_ispeed;
            public uint c_ospeed;
        }

        [DllImport("libc")]
        private static extern int tcgetattr(int fd, out Termios termios_p);

        [DllImport("libc")]
        private static extern int tcsetattr(int fd, int optional_actions, ref Termios termios_p);

        [DllImport("libc")]
        private static extern int read(int fd, byte[] buf, int count);

        private const int STDIN_FILENO = 0;
        private const int TCSANOW = 0;
        private static Termios originalTermios;

        public static void Start()
        {
            SetRawMode(true);
            Thread inputThread = new(ReadKeys)
            {
                IsBackground = true
            };
            inputThread.Start();
        }

        private static void SetRawMode(bool enable)
        {
            tcgetattr(STDIN_FILENO, out originalTermios);
            Termios raw = originalTermios;
            if (enable)
            {
                raw.c_lflag &= ~(1u | 2u | 4u | 8u | 256u); // Tắt canonical + echo + signal
            }
            else
            {
                raw = originalTermios;
            }
            tcsetattr(STDIN_FILENO, TCSANOW, ref raw);
        }

        public static void ReadKeys()
        {
            Console.WriteLine("Điều khiển:");
            Console.WriteLine("← → : Xoay trái / phải (node1)");
            Console.WriteLine("↑ ↓ : Nâng / hạ (node2)");
            Console.WriteLine("Giữ phím để chạy, nhả để dừng.");
            Console.WriteLine("C : Thoát chương trình\n");

            byte[] buf = new byte[3];

            while (_running)
            {
                int n = read(STDIN_FILENO, buf, 3);
                if (n > 0)
                {
                    if (n == 3 && buf[0] == 0x1B && buf[1] == 0x5B)
                    {
                        switch (buf[2])
                        {
                            case 0x41: _speedLift = 10; break;  // ↑
                            case 0x42: _speedLift = -10; break; // ↓
                            case 0x43: _speedRotate = 10; break; // →
                            case 0x44: _speedRotate = -10; break; // ←
                        }
                    }
                    else if (n == 1)
                    {
                        if (buf[0] == 'c' || buf[0] == 'C')
                        {
                            _running = false;
                            _speedLift = 0;
                            _speedRotate = 0;
                            Console.WriteLine("\nThoát chương trình.");
                            break;
                        }
                    }
                }
                else
                {
                    // Không có phím nào đang nhấn -> dừng
                    _speedRotate = 0;
                    _speedLift = 0;
                }

                Thread.Sleep(50); // Giảm tải CPU
            }

            SetRawMode(false);
        }

        public static (int rotate, int lift) GetSpeeds()
        {
            return (_speedRotate, _speedLift);
        }
    }

    public class ControlPDO
    {
        static int preProfileVelocity;
        static readonly int gear_ratio = 350; // tỉ số truyền cơ khí

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct CanFrame
        {
            public uint can_id;
            public byte can_dlc;
            public byte __pad1, __pad2, __pad3;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] data;
        }

        [DllImport("libc", SetLastError = true)]
        public static extern int write(int fd, ref CanFrame frame, int len);

        [DllImport("libc", SetLastError = true)]
        public static extern int read(int fd, ref CanFrame frame, int len);

        static readonly int encoder_Resolution = 10000;
        // Hàm gửi frame
        public static void SendFrame(int __sock, uint canId, byte[] payload)
        {
            CanFrame frame = new()
            {
                can_id = canId,
                can_dlc = (byte)payload.Length
            };
            frame.__pad1 = frame.__pad2 = frame.__pad3 = 0;
            frame.data = new byte[8];
            Array.Copy(payload, frame.data, payload.Length);
            // Console.WriteLine($"");

            int nbytes = write(__sock, ref frame, Marshal.SizeOf(typeof(CanFrame)));
            if (nbytes < 0)
                Console.WriteLine("Lỗi khi gửi frame!");
        }


        public static void WriteSDO_U32(int sock, uint nodeId, ushort index, byte subIndex, uint value)
        {
            uint sdoTxId = 0x600 + nodeId;
            byte[] data =
            [
                0x23, // Write 4 bytes
                (byte)(index & 0xFF),
                (byte)(index >> 8),
                subIndex,
                (byte)(value & 0xFF),
                (byte)((value >> 8) & 0xFF),
                (byte)((value >> 16) & 0xFF),
                (byte)((value >> 24) & 0xFF),
            ];
            SendFrame(sock, sdoTxId, data);
            Thread.Sleep(10);
        }

        public static void WriteSDO_U8(int sock, uint nodeId, ushort index, byte subIndex, byte value)
        {
            uint sdoTxId = 0x600 + nodeId;
            byte[] data = new byte[8];
            data[0] = 0x2F; // Write 1 byte
            data[1] = (byte)(index & 0xFF);
            data[2] = (byte)(index >> 8);
            data[3] = subIndex;
            data[4] = value;
            SendFrame(sock, sdoTxId, data);
            Thread.Sleep(10);
        }

        public static void ConfigurePDO_Default(int sock, uint nodeId)
        {
            Console.WriteLine("Configuring PDO mapping...");

            // ================== TPDO ==================
            // Disable TPDO1-4
            WriteSDO_U32(sock, nodeId, 0x1800, 1, 0x80000180 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1801, 1, 0x80000280 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1802, 1, 0x80000380 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1803, 1, 0x80000480 + nodeId);

            // TPDO1: Statusword(6041:00/16), ModeDisplay(6061:00/8), Error(603F:00/16)
            WriteSDO_U8(sock, nodeId, 0x1A00, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A00, 1, 0x60410010);
            WriteSDO_U32(sock, nodeId, 0x1A00, 2, 0x60610008);
            WriteSDO_U32(sock, nodeId, 0x1A00, 3, 0x603F0010);
            WriteSDO_U8(sock, nodeId, 0x1A00, 0, 3);
            WriteSDO_U32(sock, nodeId, 0x1800, 1, 0x00000180 + nodeId);

            // TPDO2: Position(6064:00/32)
            WriteSDO_U8(sock, nodeId, 0x1A01, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A01, 1, 0x60640020);
            WriteSDO_U8(sock, nodeId, 0x1A01, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1801, 1, 0x00000280 + nodeId);

            // TPDO3: Velocity(606C:00/32)
            WriteSDO_U8(sock, nodeId, 0x1A02, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A02, 1, 0x606C0020);
            WriteSDO_U8(sock, nodeId, 0x1A02, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1802, 1, 0x00000380 + nodeId);

            // TPDO4: Position(6064:00/32), Velocity(606C:00/32)
            WriteSDO_U8(sock, nodeId, 0x1A03, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A03, 1, 0x60640020);
            WriteSDO_U32(sock, nodeId, 0x1A03, 2, 0x606C0020);
            WriteSDO_U8(sock, nodeId, 0x1A03, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1803, 1, 0x00000480 + nodeId);

            // ================== RPDO ==================
            // Disable RPDO1-4
            WriteSDO_U32(sock, nodeId, 0x1400, 1, 0x80000200 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1401, 1, 0x80000300 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1402, 1, 0x80000400 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1403, 1, 0x80000500 + nodeId);

            // RPDO1: Controlword(6040:00/16)
            WriteSDO_U8(sock, nodeId, 0x1600, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1600, 1, 0x60400010);
            WriteSDO_U8(sock, nodeId, 0x1600, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1400, 1, 0x00000200 + nodeId);

            // RPDO2: Controlword(6040:00/16), Target Position(607A:00/32)
            WriteSDO_U8(sock, nodeId, 0x1601, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1601, 1, 0x60400010);
            WriteSDO_U32(sock, nodeId, 0x1601, 2, 0x607A0020);
            WriteSDO_U8(sock, nodeId, 0x1601, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1401, 1, 0x00000300 + nodeId);

            // RPDO3: Controlword(6040:00/16), Target Velocity(60FF:00/32)
            WriteSDO_U8(sock, nodeId, 0x1602, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1602, 1, 0x60400010);
            WriteSDO_U32(sock, nodeId, 0x1602, 2, 0x60FF0020);
            WriteSDO_U8(sock, nodeId, 0x1602, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1402, 1, 0x00000400 + nodeId);

            // RPDO4: Physical output(60FE:01/32)
            WriteSDO_U8(sock, nodeId, 0x1603, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1603, 1, 0x60FE0120);
            WriteSDO_U8(sock, nodeId, 0x1603, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1403, 1, 0x00000500 + nodeId);

            Console.WriteLine("PDO mapping configured to default.");
        }
        // RPDO1: Controlword
        public static void SendControlword(int __sock, uint nodeId, ushort controlword)
        {
            uint rpdo1Id = 0x200 + nodeId;
            byte[] data = new byte[8];
            data[0] = (byte)(controlword & 0xFF);
            data[1] = (byte)(controlword >> 8);
            SendFrame(__sock, rpdo1Id, data);
            Console.WriteLine($"[TPDO1] Controlword = 0x{controlword:X4}");
        }

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

        public static void SendPosition(int __sock, uint nodeId, int targetPos, int profileVelocity)
        {
            uint rpdo2Id = 0x300 + nodeId;   // RPDO2: Controlword + Target Position
            uint sdoTxId = 0x600 + nodeId;   // SDO Tx

            // --- nếu cần update Profile Velocity ---
            if (profileVelocity != preProfileVelocity)
            {
                byte[] velData = new byte[8];
                velData[0] = 0x23; // SDO download 4 bytes
                velData[1] = 0x81;
                velData[2] = 0x60;
                velData[3] = 0x00;
                long _profile_velocity = (((long)profileVelocity * 512 * encoder_Resolution) / 1875);
                byte[] vbytes = BitConverter.GetBytes(_profile_velocity);
                Array.Copy(vbytes, 0, velData, 4, 4);
                for (int i = 0; i < 3; i++)
                {
                    SendFrame(__sock, sdoTxId, velData);
                }
                preProfileVelocity = profileVelocity;
                Thread.Sleep(50);
            }

            // --- Chuẩn bị frame RPDO2 ---
            ushort baseCw = 0x000F; // Operation enabled
            byte[] posBytes = BitConverter.GetBytes(targetPos);

            // Lần 1: gửi controlword + pos với bit4=1, bit5=1
            ushort cw_setpoint = (ushort)(baseCw | (1 << 4) | (1 << 5));
            byte[] data = new byte[8];
            data[0] = (byte)(cw_setpoint & 0xFF);
            data[1] = (byte)(cw_setpoint >> 8);
            Array.Copy(posBytes, 0, data, 2, 4);
            SendFrame(__sock, rpdo2Id, data);

            // Lần 2: clear bit4, giữ bit5
            Thread.Sleep(10);
            ushort cw_clear = (ushort)(baseCw | (1 << 5));
            byte[] data2 = new byte[8];
            data2[0] = (byte)(cw_clear & 0xFF);
            data2[1] = (byte)(cw_clear >> 8);
            Array.Copy(posBytes, 0, data2, 2, 4);
            SendFrame(__sock, rpdo2Id, data2);

            Console.WriteLine($"[RPDO2] TargetPos={targetPos}, ProfileVel={profileVelocity}");
        }

        public static void SendVelocity(int __sock, uint nodeId, int velocity, ushort controlword = 0x000F)
        {
            uint rpdo3Id = 0x400 + nodeId;   // RPDO3
            byte[] data = new byte[6];

            data[0] = (byte)(controlword & 0xFF);
            data[1] = (byte)(controlword >> 8);
            long _velocity = (((long)velocity * 512 * encoder_Resolution) / 1875);
            byte[] velBytes = BitConverter.GetBytes(_velocity);
            Array.Copy(velBytes, 0, data, 2, 4);

            SendFrame(__sock, rpdo3Id, data);
            Console.WriteLine($"[TPDO3] Velocity target = {_velocity}");
        }

        // Lắng nghe TPDO (status, velocity actual)
        public static void ListenTPDO(int __sock, uint nodeId)
        {
            while (ControlSpeed._running)
            {
                CanFrame rx = new();
                int nbytes = read(__sock, ref rx, Marshal.SizeOf(typeof(CanFrame)));
                if (nbytes > 0)
                {
                    if (rx.can_id == 0x180 + nodeId) // TPDO1
                    {
                        ushort status = BitConverter.ToUInt16(rx.data, 0);
                        Console.WriteLine($"[RPDO1] Statusword = 0x{status:X4}");
                    }
                    if (rx.can_id == 0x280 + nodeId)
                    {
                        int pos = BitConverter.ToInt32(rx.data, 0);
                        Console.WriteLine($"[RPDO2: Positon actual = {pos}");
                    }
                    if (rx.can_id == 0x380 + nodeId) // TPDO3
                    {
                        long _vel = BitConverter.ToInt32(rx.data, 0);
                        long vel = (_vel * 1875) / (512 * encoder_Resolution * gear_ratio);
                        Console.WriteLine($"[RPDO3] Velocity actual = {vel}");
                    }
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

        public static void EnableMotorVelocityMode(int ___sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + (uint)_nodeId;
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
        }

        public static void EnableMotorPositionMode(int ___sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + (uint)_nodeId;
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
        }

        public static void EnableMotorHomeMode(int ___sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + (uint)_nodeId;
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(50);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            ControlPDO.SendFrame(___sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
        }
    }
    class Program
    {
        const int AF_CAN = 29;
        const int SOCK_RAW = 3;
        const int CAN_RAW = 1;
        const uint SIOCGIFINDEX = 0x8933;

        [StructLayout(LayoutKind.Sequential)]
        struct IfReq
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] ifr_name;
            public int ifr_ifindex;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct SockAddrCan
        {
            public ushort can_family;
            public int can_ifindex;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] _pad;
        }

        [DllImport("libc", SetLastError = true)]
        static extern int socket(int domain, int type, int protocol);

        [DllImport("libc", SetLastError = true)]
        static extern int ioctl(int fd, uint request, ref IfReq ifr);

        [DllImport("libc", SetLastError = true)]
        static extern int bind(int fd, ref SockAddrCan addr, int addrlen);

        static int sock;
        static readonly uint nodeId = 1;
        static readonly uint nodeId2 = 2;
        static readonly int gear_ratio_rotate_motor = 350;
        static readonly int convert_lift_param = 33; //lift module: 1mm/s = 33 rpm
        static bool registerShutdowAction = true;

        static void Main(string[] args)
        {

            // Thiết lập CAN
            Console.Write("\nNhập tên interface CAN (mặc định can0): ");
            string? interfaceName = Console.ReadLine();
            if (string.IsNullOrWhiteSpace(interfaceName)) interfaceName = "can0";

            Console.Write("Nhập baudrate (vd: 500000): ");
            string? baudStr = Console.ReadLine();
            if (!int.TryParse(baudStr, out int baudrate)) baudrate = 500000;

            if (!StartCanOpen(interfaceName, baudrate))
            {
                Console.WriteLine("❌ Không thể khởi tạo CAN interface. Thoát chương trình.");
                return;
            }

            ControlPDO.SendFrame(sock, 0x000, [0x01, (byte)nodeId]);
            ControlPDO.SendFrame(sock, 0x000, [0x01, (byte)nodeId2]);
            Console.WriteLine("Sent NMT Start to node " + nodeId);

            ControlPDO.ConfigurePDO_Default(sock, nodeId);
            ControlPDO.ConfigurePDO_Default(sock, nodeId2);

            ControlPDO.EnableMotorVelocityMode(sock, nodeId);
            ControlPDO.EnableMotorVelocityMode(sock, nodeId2);

            // ControlPDO.EnableMotorPositionMode(sock, nodeId);
            // ControlPDO.EnableMotorPositionMode(sock, nodeId2);

            ControlSpeed.Start();

            if (ControlSpeed._running)
            {
                Thread tpdoThread = new(() => ControlPDO.ListenTPDO(sock, (uint)nodeId));
                tpdoThread.Start();
            }
            while (ControlSpeed._running)
            {
                if (registerShutdowAction)
                {
                    Console.CancelKeyPress += new ConsoleCancelEventHandler(OnExit);
                    AppDomain.CurrentDomain.ProcessExit += new EventHandler(OnProcessExit);
                    registerShutdowAction = false;
                }

                var (speed_rotation, speed_lift) = ControlSpeed.GetSpeeds();

                // Quy đổi sang tốc độ cơ học
                int speed_rotation_rpm = speed_rotation * gear_ratio_rotate_motor;
                int speed_lift_rpm = speed_lift * convert_lift_param;

                // Node1: xoay, Node2: nâng hạ
                ControlPDO.SendVelocity(sock, nodeId, speed_rotation_rpm);
                ControlPDO.SendVelocity(sock, nodeId2, speed_lift_rpm);

                Thread.Sleep(200);
            }


        }
        static string ExecuteCommand(string command)
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
            catch (Exception ex)
            {
                return $"Lỗi khi chạy lệnh: {ex.Message}";
            }
        }
        static bool StartCanOpen(string interfaceName, int baudrate)
        {
            Console.WriteLine($"Thiết lập CAN {interfaceName} với baudrate {baudrate}");
            ExecuteCommand($"sudo ip link set {interfaceName} down");
            ExecuteCommand($"sudo ip link set {interfaceName} type can bitrate {baudrate}");
            var result = ExecuteCommand($"sudo ip link set {interfaceName} up");

            if (result.Contains("error", StringComparison.OrdinalIgnoreCase))
            {
                Console.WriteLine($"[ERROR] Lỗi thiết lập CAN: {result}");
                return false;
            }

            sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (sock < 0)
            {
                Console.WriteLine("Không tạo được socket CAN!");
                return false;
            }

            IfReq ifr = new()
            {
                ifr_name = Encoding.ASCII.GetBytes($"{interfaceName}\0".PadRight(16, '\0')),
                ifr_ifindex = 0
            };

            if (ioctl(sock, SIOCGIFINDEX, ref ifr) < 0)
            {
                Console.WriteLine($"Không lấy được ifindex của {interfaceName}!");
                return false;
            }

            SockAddrCan addr = new()
            {
                can_family = AF_CAN,
                can_ifindex = ifr.ifr_ifindex,
                _pad = new byte[8]
            };

            if (bind(sock, ref addr, Marshal.SizeOf(typeof(SockAddrCan))) < 0)
            {
                Console.WriteLine($"Không bind được socket tới {interfaceName}!");
                return false;
            }

            Console.WriteLine($"✅ SocketCAN opened & bound to {interfaceName} (fd={sock})");
            return true;
        }

        static void OnExit(object? sender, ConsoleCancelEventArgs args)
        {
            Console.WriteLine("Nhận Ctrl+C, gửi Velocity = 0...");
            ControlSpeed._running = false;
            ControlPDO.SendVelocity(sock, nodeId, 0);
            Thread.Sleep(200); // đảm bảo gửi đi
        }

        static void OnProcessExit(object? sender, EventArgs e)
        {
            Console.WriteLine("ProcessExit, gửi Velocity = 0...");
            ControlSpeed._running = false;
            ControlPDO.SendVelocity(sock, nodeId, 0);
            Thread.Sleep(200);
        }
    }

}

