using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace dieukhienxoaynang
{
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
        static readonly uint nodeId = 1; // Động cơ xoay
        static readonly uint nodeId2 = 2; // Động cơ nâng
        static readonly int gear_ratio_rotate_motor = 350;
        static readonly int convert_lift_param = 33; // lift module: 1mm/s = 33 rpm
        static bool registerShutdowAction = true;
        static int controlMode = 0; // 1: xoay, 2: nâng, 3: cả hai

        static void Main(string[] args)
        {
            ArgumentNullException.ThrowIfNull(args);
            Console.WriteLine("====================================");
            Console.WriteLine("  Điều khiển Xoay và Nâng qua CAN  ");
            Console.WriteLine("====================================\n");

            Console.WriteLine("Chọn chế độ điều khiển:");
            Console.WriteLine("  1: Chỉ xoay (Node 1)");
            Console.WriteLine("  2: Chỉ nâng (Node 2)");
            Console.WriteLine("  3: Cả xoay và nâng");
            Console.Write("Nhập số (1-3): ");
            while (!int.TryParse(Console.ReadLine(), out controlMode) || controlMode < 1 || controlMode > 3)
            {
                Console.Write("Vui lòng nhập số hợp lệ (1-3): ");
            }
            Console.WriteLine($"Chế độ điều khiển: {(controlMode == 1 ? "Chỉ xoay" : controlMode == 2 ? "Chỉ nâng" : "Cả xoay và nâng")}");

            StartCanOpen();

            if (sock < 0) return; // Thoát nếu không mở được socket CAN

            // Kiểm tra trạng thái động cơ
            ushort status1 = ControlSDO.ReadStatusword(sock, nodeId);
            ushort status2 = ControlSDO.ReadStatusword(sock, nodeId2);
            Console.WriteLine($"Trạng thái động cơ Node 1: 0x{status1:X4}");
            Console.WriteLine($"Trạng thái động cơ Node 2: 0x{status2:X4}");

            // Khởi động NMT
            ControlPDO.SendFrame(sock, 0x000, [0x01, (byte)nodeId]);
            ControlPDO.SendFrame(sock, 0x000, [0x01, (byte)nodeId2]);
            Console.WriteLine($"Gửi NMT Start: Node {nodeId} và Node {nodeId2}");

            // Cấu hình PDO
            ControlPDO.ConfigurePDO_Default(sock, nodeId);
            ControlPDO.ConfigurePDO_Default(sock, nodeId2);

            // Kích hoạt Velocity Mode
            ControlPDO.EnableMotorVelocityMode(sock, nodeId);
            ControlPDO.EnableMotorVelocityMode(sock, nodeId2);

            ControlSpeed.Start();

            if (ControlSpeed._running)
            {
                Thread tpdoThread = new(() => ControlPDO.ListenTPDO(sock, (uint)nodeId));
                Thread tpdoThread2 = new(() => ControlPDO.ListenTPDO(sock, (uint)nodeId2));
                tpdoThread.Start();
                tpdoThread2.Start();
            }

            while (ControlSpeed._running)
            {
                if (registerShutdowAction)
                {
                    Console.CancelKeyPress += new ConsoleCancelEventHandler(OnExit);
                    AppDomain.CurrentDomain.ProcessExit += new EventHandler(OnProcessExit);
                    registerShutdowAction = false;
                }

                int speed_rotation = ControlSpeed.GetSpeed() * gear_ratio_rotate_motor;
                int speed_lift = ControlSpeed.GetSpeed() * convert_lift_param;

                if (controlMode == 1) // Chỉ xoay
                {
                    ControlPDO.SendVelocity(sock, (uint)nodeId, speed_rotation);
                    ControlPDO.SendVelocity(sock, (uint)nodeId2, 0);
                    Console.WriteLine($"Chế độ: Chỉ xoay, Tốc độ xoay: {speed_rotation}, Tốc độ nâng: 0");
                }
                else if (controlMode == 2) // Chỉ nâng
                {
                    ControlPDO.SendVelocity(sock, (uint)nodeId, 0);
                    ControlPDO.SendVelocity(sock, (uint)nodeId2, speed_lift);
                    Console.WriteLine($"Chế độ: Chỉ nâng, Tốc độ xoay: 0, Tốc độ nâng: {speed_lift}");
                }
                else // Cả xoay và nâng
                {
                    ControlPDO.SendVelocity(sock, (uint)nodeId, speed_rotation);
                    ControlPDO.SendVelocity(sock, (uint)nodeId2, speed_lift);
                    Console.WriteLine($"Chế độ: Cả xoay và nâng, Tốc độ xoay: {speed_rotation}, Tốc độ nâng: {speed_lift}");
                }

                Thread.Sleep(1000);
            }
        }

        static void StartCanOpen()
        {
            try
            {
                Console.WriteLine("Thiết lập giao diện CAN: can0...");
                RunCommand("ip", "link set can0 down");
                RunCommand("ip", "link set can0 type can bitrate 500000");
                RunCommand("ip", "link set can0 up");
                Console.WriteLine("Giao diện CAN: can0 đã được thiết lập.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Lỗi thiết lập giao diện CAN: {ex.Message}");
                return;
            }

            sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (sock < 0)
            {
                Console.WriteLine("Lỗi tạo socket CAN: Không thể mở socket.");
                return;
            }

            IfReq ifr = new()
            {
                ifr_name = Encoding.ASCII.GetBytes("can0\0".PadRight(16, '\0')),
                ifr_ifindex = 0
            };
            if (ioctl(sock, SIOCGIFINDEX, ref ifr) < 0)
            {
                Console.WriteLine("Lỗi lấy ifindex: Không tìm thấy can0.");
                return;
            }

            SockAddrCan addr = new()
            {
                can_family = AF_CAN,
                can_ifindex = ifr.ifr_ifindex,
                _pad = new byte[8]
            };
            if (bind(sock, ref addr, Marshal.SizeOf(typeof(SockAddrCan))) < 0)
            {
                Console.WriteLine("Lỗi bind socket: Không thể gắn với can0.");
                return;
            }

            Console.WriteLine($"Socket CAN: Đã mở và gắn với can0 (fd={sock}).");
        }

        static void RunCommand(string command, string arguments)
        {
            Process process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = command,
                    Arguments = arguments,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            try
            {
                process.Start();
                string error = process.StandardError.ReadToEnd();
                process.WaitForExit();

                if (process.ExitCode != 0)
                {
                    throw new Exception(error);
                }
            }
            catch (Exception ex)
            {
                throw new Exception($"Lệnh thất bại: {command} {arguments}, Lỗi: {ex.Message}");
            }
        }

        static void OnExit(object? sender, ConsoleCancelEventArgs args)
        {
            Console.WriteLine("Nhận Ctrl+C: Gửi tốc độ = 0...");
            ControlSpeed._running = false;
            ControlPDO.SendVelocity(sock, nodeId, 0);
            ControlPDO.SendVelocity(sock, nodeId2, 0);
            Thread.Sleep(200);
        }

        static void OnProcessExit(object? sender, EventArgs e)
        {
            Console.WriteLine("Thoát chương trình: Gửi tốc độ = 0...");
            ControlSpeed._running = false;
            ControlPDO.SendVelocity(sock, nodeId, 0);
            ControlPDO.SendVelocity(sock, nodeId2, 0);
            Thread.Sleep(200);
        }
    }

    public class ControlSpeed
    {
        public static int _speed = 0;
        public static bool _running = true;

        public static void Start()
        {
            Thread inputThread = new(ReadKeys)
            {
                IsBackground = true
            };
            inputThread.Start();
        }

        public static int GetSpeed()
        {
            return _speed;
        }

        public static void ReadKeys()
        {
            Console.WriteLine("\nHướng dẫn điều khiển:");
            Console.WriteLine("  Mũi tên Lên: Tốc độ +10");
            Console.WriteLine("  Mũi tên Xuống: Tốc độ -10");
            Console.WriteLine("  Phím S: Dừng (tốc độ = 0)");
            Console.WriteLine("  Phím C: Thoát chương trình\n");

            while (_running)
            {
                if (Console.KeyAvailable)
                {
                    var key = Console.ReadKey(true).Key;

                    if (key == ConsoleKey.UpArrow)
                    {
                        _speed = 10;
                        Console.WriteLine("Tốc độ: +10");
                    }
                    else if (key == ConsoleKey.DownArrow)
                    {
                        _speed = -10;
                        Console.WriteLine("Tốc độ: -10");
                    }
                    else if (key == ConsoleKey.S)
                    {
                        _speed = 0;
                        Console.WriteLine("Tốc độ: 0 (Dừng)");
                    }
                    else if (key == ConsoleKey.C)
                    {
                        _speed = 0;
                        _running = false;
                        Console.WriteLine("Thoát: Chương trình dừng.");
                    }
                }

                Thread.Sleep(100);
            }
        }
    }
    public class ControlPDO
    {
        static int preProfileVelocity;

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
        static readonly int gear_ratio = 350;

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

            int nbytes = write(__sock, ref frame, Marshal.SizeOf(typeof(CanFrame)));
            if (nbytes < 0)
            {
                int error = Marshal.GetLastWin32Error();
                Console.WriteLine($"Lỗi gửi frame CAN: ID=0x{canId:X3}, Payload={BitConverter.ToString(payload)}, Mã lỗi={error}");
            }
        }

        public static void WriteSDO_U32(int sock, uint nodeId, ushort index, byte subIndex, uint value)
        {
            uint sdoTxId = 0x600 + nodeId;
            byte[] data =
            [
                0x23,
                (byte)(index & 0xFF),
                (byte)(index >> 8),
                subIndex,
                (byte)(value & 0xFF),
                (byte)((value >> 8) & 0xFF),
                (byte)((value >> 16) & 0xFF),
                (byte)((value >> 24) & 0xFF),
            ];
            SendFrame(sock, sdoTxId, data);
            Thread.Sleep(100);
        }

        public static void WriteSDO_U8(int sock, uint nodeId, ushort index, byte subIndex, byte value)
        {
            uint sdoTxId = 0x600 + nodeId;
            byte[] data = new byte[8];
            data[0] = 0x2F;
            data[1] = (byte)(index & 0xFF);
            data[2] = (byte)(index >> 8);
            data[3] = subIndex;
            data[4] = value;
            SendFrame(sock, sdoTxId, data);
            Thread.Sleep(100);
        }

        public static void ConfigurePDO_Default(int sock, uint nodeId)
        {
            Console.WriteLine($"Cấu hình PDO: Node {nodeId}...");
            WriteSDO_U32(sock, nodeId, 0x1800, 1, 0x80000180 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1801, 1, 0x80000280 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1802, 1, 0x80000380 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1803, 1, 0x80000480 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1A00, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A00, 1, 0x60410010);
            WriteSDO_U32(sock, nodeId, 0x1A00, 2, 0x60610008);
            WriteSDO_U32(sock, nodeId, 0x1A00, 3, 0x603F0010);
            WriteSDO_U8(sock, nodeId, 0x1A00, 0, 3);
            WriteSDO_U32(sock, nodeId, 0x1800, 1, 0x00000180 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1A01, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A01, 1, 0x60640020);
            WriteSDO_U8(sock, nodeId, 0x1A01, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1801, 1, 0x00000280 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1A02, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A02, 1, 0x606C0020);
            WriteSDO_U8(sock, nodeId, 0x1A02, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1802, 1, 0x00000380 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1A03, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1A03, 1, 0x60640020);
            WriteSDO_U32(sock, nodeId, 0x1A03, 2, 0x606C0020);
            WriteSDO_U8(sock, nodeId, 0x1A03, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1803, 1, 0x00000480 + nodeId);

            WriteSDO_U32(sock, nodeId, 0x1400, 1, 0x80000200 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1401, 1, 0x80000300 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1402, 1, 0x80000400 + nodeId);
            WriteSDO_U32(sock, nodeId, 0x1403, 1, 0x80000500 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1600, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1600, 1, 0x60400010);
            WriteSDO_U8(sock, nodeId, 0x1600, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1400, 1, 0x00000200 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1601, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1601, 1, 0x60400010);
            WriteSDO_U32(sock, nodeId, 0x1601, 2, 0x607A0020);
            WriteSDO_U8(sock, nodeId, 0x1601, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1401, 1, 0x00000300 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1602, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1602, 1, 0x60400010);
            WriteSDO_U32(sock, nodeId, 0x1602, 2, 0x60FF0020);
            WriteSDO_U8(sock, nodeId, 0x1602, 0, 2);
            WriteSDO_U32(sock, nodeId, 0x1402, 1, 0x00000400 + nodeId);

            WriteSDO_U8(sock, nodeId, 0x1603, 0, 0);
            WriteSDO_U32(sock, nodeId, 0x1603, 1, 0x60FE0120);
            WriteSDO_U8(sock, nodeId, 0x1603, 0, 1);
            WriteSDO_U32(sock, nodeId, 0x1403, 1, 0x00000500 + nodeId);

            Console.WriteLine($"Cấu hình PDO: Node {nodeId} hoàn tất.");
        }

        public static void SendControlword(int __sock, uint nodeId, ushort controlword)
        {
            uint rpdo1Id = 0x200 + nodeId;
            byte[] data = new byte[8];
            data[0] = (byte)(controlword & 0xFF);
            data[1] = (byte)(controlword >> 8);
            SendFrame(__sock, rpdo1Id, data);
            Console.WriteLine($"Gửi Controlword: Node {nodeId}, Giá trị=0x{controlword:X4}");
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
                ushort status = BitConverter.ToUInt16(rxFrame.data, 4);
                Console.WriteLine($"Đọc Statusword: Node {nodeId}, Giá trị=0x{status:X4}");
                return status;
            }
            Console.WriteLine($"Lỗi đọc Statusword: Node {nodeId}, Không nhận được phản hồi.");
            return 0;
        }

        public static void SendPosition(int __sock, uint nodeId, int targetPos, int profileVelocity)
        {
            uint rpdo2Id = 0x300 + nodeId;
            uint sdoTxId = 0x600 + nodeId;

            if (profileVelocity != preProfileVelocity)
            {
                byte[] velData = new byte[8];
                velData[0] = 0x23;
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
                Thread.Sleep(100);
            }

            ushort baseCw = 0x000F;
            byte[] posBytes = BitConverter.GetBytes(targetPos);

            ushort cw_setpoint = (ushort)(baseCw | (1 << 4) | (1 << 5));
            byte[] data = new byte[8];
            data[0] = (byte)(cw_setpoint & 0xFF);
            data[1] = (byte)(cw_setpoint >> 8);
            Array.Copy(posBytes, 0, data, 2, 4);
            SendFrame(__sock, rpdo2Id, data);

            Thread.Sleep(100);
            ushort cw_clear = (ushort)(baseCw | (1 << 5));
            byte[] data2 = new byte[8];
            data2[0] = (byte)(cw_clear & 0xFF);
            data2[1] = (byte)(cw_clear >> 8);
            Array.Copy(posBytes, 0, data2, 2, 4);
            SendFrame(__sock, rpdo2Id, data2);

            Console.WriteLine($"Gửi vị trí: Node {nodeId}, Vị trí={targetPos}, Tốc độ cấu hình={profileVelocity}");
        }

        public static void SendVelocity(int __sock, uint nodeId, int velocity, ushort controlword = 0x000F)
        {
            uint rpdo3Id = 0x400 + nodeId;
            byte[] data = new byte[6];
            data[0] = (byte)(controlword & 0xFF);
            data[1] = (byte)(controlword >> 8);
            long _velocity = (((long)velocity * 512 * encoder_Resolution) / 1875);
            byte[] velBytes = BitConverter.GetBytes(_velocity);
            Array.Copy(velBytes, 0, data, 2, 4);

            SendFrame(__sock, rpdo3Id, data);
            Console.WriteLine($"Gửi tốc độ PDO: Node {nodeId}, Tốc độ={_velocity}");
        }

        public static void ListenTPDO(int __sock, uint nodeId)
        {
            while (ControlSpeed._running)
            {
                CanFrame rx = new();
                int nbytes = read(__sock, ref rx, Marshal.SizeOf(typeof(CanFrame)));
                if (nbytes > 0)
                {
                    if (rx.can_id == 0x180 + nodeId)
                    {
                        ushort status = BitConverter.ToUInt16(rx.data, 0);
                        Console.WriteLine($"Nhận TPDO1: Node {nodeId}, Statusword=0x{status:X4}");
                    }
                    if (rx.can_id == 0x280 + nodeId)
                    {
                        int pos = BitConverter.ToInt32(rx.data, 0);
                        Console.WriteLine($"Nhận TPDO2: Node {nodeId}, Vị trí thực tế={pos}");
                    }
                    if (rx.can_id == 0x380 + nodeId)
                    {
                        long _vel = BitConverter.ToInt32(rx.data, 0);
                        long vel = (_vel * 1875) / (512 * encoder_Resolution * gear_ratio);
                        Console.WriteLine($"Nhận TPDO3: Node {nodeId}, Tốc độ thực tế={vel}");
                    }
                    if (rx.can_id == 0x480 + nodeId)
                    {
                        long _pos = BitConverter.ToInt32(rx.data, 0);
                        long _vel = BitConverter.ToInt32(rx.data, 4);
                        Console.WriteLine($"Nhận TPDO4: Node {nodeId}, Vị trí thực tế={_pos}, Tốc độ thực tế={_vel}");
                    }
                }
                Thread.Sleep(5);
            }
        }

        public static void EnableMotorVelocityMode(int __sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + (uint)_nodeId;
            SendFrame(__sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Console.WriteLine($"Kích hoạt chế độ Velocity: Node {_nodeId} hoàn tất.");
        }

        public static void EnableMotorPositionMode(int __sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + (uint)_nodeId;
            SendFrame(__sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Console.WriteLine($"Kích hoạt chế độ Position: Node {_nodeId} hoàn tất.");
        }

        public static void EnableMotorHomeMode(int __sock, uint _nodeId)
        {
            uint sdoTxId = 0x600 + (uint)_nodeId;
            SendFrame(__sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x40, 0x61, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(__sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]);
            Console.WriteLine($"Kích hoạt chế độ Home: Node {_nodeId} hoàn tất.");
        }
    }

    public class ControlSDO
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

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        struct CanFrame
        {
            public uint can_id;
            public byte can_dlc;
            public byte __pad1, __pad2, __pad3;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public byte[] data;
        }

        [DllImport("libc", SetLastError = true)]
        static extern int socket(int domain, int type, int protocol);
        [DllImport("libc", SetLastError = true)]
        static extern int ioctl(int fd, uint request, ref IfReq ifr);
        [DllImport("libc", SetLastError = true)]
        static extern int bind(int fd, ref SockAddrCan addr, int addrlen);
        [DllImport("libc", SetLastError = true)]
        static extern int write(int fd, ref CanFrame frame, int len);
        [DllImport("libc", SetLastError = true)]
        static extern int read(int fd, ref CanFrame frame, int len);

        public static int OpenSocket(string ifname)
        {
            int sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (sock < 0)
            {
                Console.WriteLine("Lỗi socket CAN: Không thể mở socket.");
                throw new Exception("Cannot open socket!");
            }

            IfReq ifr = new()
            {
                ifr_name = Encoding.ASCII.GetBytes(ifname.PadRight(16, '\0')),
                ifr_ifindex = 0
            };
            if (ioctl(sock, SIOCGIFINDEX, ref ifr) < 0)
            {
                Console.WriteLine("Lỗi ifindex: Không tìm thấy giao diện CAN.");
                throw new Exception("Cannot get ifindex!");
            }

            SockAddrCan addr = new()
            {
                can_family = AF_CAN,
                can_ifindex = ifr.ifr_ifindex,
                _pad = new byte[8]
            };
            if (bind(sock, ref addr, Marshal.SizeOf(typeof(SockAddrCan))) < 0)
            {
                Console.WriteLine("Lỗi bind CAN: Không thể gắn socket.");
                throw new Exception("Cannot bind socket!");
            }

            return sock;
        }

        public static void SendFrame(int sock, uint canId, byte[] payload)
        {
            CanFrame frame = new();
            frame.can_id = canId;
            frame.can_dlc = (byte)payload.Length;
            frame.__pad1 = frame.__pad2 = frame.__pad3 = 0;
            frame.data = new byte[8];
            Array.Copy(payload, frame.data, payload.Length);
            int nbytes = write(sock, ref frame, Marshal.SizeOf(typeof(CanFrame)));
            if (nbytes < 0)
            {
                int error = Marshal.GetLastWin32Error();
                Console.WriteLine($"Lỗi gửi frame CAN: ID=0x{canId:X3}, Payload={BitConverter.ToString(payload)}, Mã lỗi={error}");
            }
        }

        public static ushort ReadStatusword(int sock, uint nodeId)
        {
            uint sdoTxId = 0x600 + nodeId;
            uint sdoRxId = 0x580 + nodeId;
            byte[] req = [0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0];
            SendFrame(sock, sdoTxId, req);

            CanFrame rxFrame = new();
            int n = read(sock, ref rxFrame, Marshal.SizeOf(typeof(CanFrame)));
            if (n > 0 && rxFrame.can_id == sdoRxId)
            {
                return BitConverter.ToUInt16(rxFrame.data, 4);
            }
            Console.WriteLine($"Lỗi đọc Statusword: Node {nodeId}, Không nhận được phản hồi.");
            return 0;
        }

        public static void EnsureMotorEnabled(int sock, uint nodeId)
        {
            uint sdoTxId = 0x600 + nodeId;
            ushort status = ReadStatusword(sock, nodeId);
            if ((status & 0x04) != 0)
            {
                Console.WriteLine($"Động cơ Node {nodeId}: Đã kích hoạt.");
                return;
            }

            SendFrame(sock, sdoTxId, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            SendFrame(sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);
            Console.WriteLine($"Động cơ Node {nodeId}: Đã gửi lệnh kích hoạt.");
        }

        public static void SetVelocitySafe(int sock, uint nodeId, int velocity)
        {
            uint sdoTxId = 0x600 + nodeId;
            byte[] velReset = BitConverter.GetBytes(0);
            SendFrame(sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, velReset[0], velReset[1], velReset[2], velReset[3]]);
            SendFrame(sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]);
            Thread.Sleep(100);

            byte[] velBytes = BitConverter.GetBytes(velocity);
            SendFrame(sock, sdoTxId, [0x23, 0xFF, 0x60, 0x00, velBytes[0], velBytes[1], velBytes[2], velBytes[3]]);
            SendFrame(sock, sdoTxId, [0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00]);
            Console.WriteLine($"Gửi tốc độ SDO: Node {nodeId}, Tốc độ={velocity}");
        }
    }

}