using SkiaSharp;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.Intrinsics;
using System.Text;
using System.Text.Json.Serialization;
using System.Numerics;
using Microsoft.AspNetCore.Identity;
using Org.BouncyCastle.Asn1.Mozilla;

namespace eft_dma_radar
{
    // Small & Miscellaneous Classes/Enums Go here

    #region Program Classes
    /// <summary>
    /// Custom Debug Stopwatch class to measure performance.
    /// </summary>
    public class DebugStopwatch
    {
        private readonly Stopwatch _sw;
        private readonly string _name;

        /// <summary>
        /// Constructor. Starts stopwatch.
        /// </summary>
        /// <param name="name">(Optional) Name of stopwatch.</param>
        public DebugStopwatch(string name = null)
        {
            _name = name;
            _sw = new Stopwatch();
            _sw.Start();
        }

        /// <summary>
        /// End stopwatch and display result to Debug Output.
        /// </summary>
        public void Stop()
        {
            _sw.Stop();
            TimeSpan ts = _sw.Elapsed;
            Debug.WriteLine($"{_name} Stopwatch Runtime: {ts.Ticks} ticks");
        }
    }

    public class ItemAnimation
    {
        public LootItem Item { get; set; }
        public float AnimationTime { get; set; }
        public float MaxAnimationTime { get; set; } = 1f;
        public int RepetitionCount { get; set; } = 1;
        public int MaxRepetitions { get; set; }

        public ItemAnimation(LootItem item)
        {
            Item = item;
            AnimationTime = 0f;
            RepetitionCount = 0;
        }
    }

    public class Hotkey
    {
        public string Action { get; set; }
        public Keys Key { get; set; }
        public HotkeyType Type { get; set; }
    }

    public class HotkeyKey
    {
        public string Name { get; }
        public Keys Key { get; }

        public HotkeyKey(string name, Keys key)
        {
            this.Name = name;
            this.Key = key;
        }

        public override string ToString()
        {
            return this.Name;
        }
    }
    #endregion

    #region Custom EFT Classes
    public class ThermalSettings
    {
        public float ColorCoefficient { get; set; }
        public float MinTemperature { get; set; }
        public float RampShift { get; set; }
        public int ColorScheme { get; set; }

        public ThermalSettings(float colorCoefficient, float minTemperature, float rampShift, int colorScheme)
        {
            this.ColorCoefficient = colorCoefficient;
            this.MinTemperature = minTemperature;
            this.RampShift = rampShift;
            this.ColorScheme = colorScheme;
        }
    }

    public class WorldSettings
    {
        public bool Fog { get; set; }
        public bool Rain { get; set; }
        public bool Clouds { get; set; }
        public bool Shadows { get; set; }
        public bool Sun { get; set; }
        public bool Moon { get; set; }
        public bool FreezeTime { get; set; }

        public bool SunLight { get; set; }
        public bool MoonLight { get; set; }
        public int SunLightIntensity { get; set; }
        public int MoonLightIntensity { get; set; }
        public int TimeOfDay { get; set; }

        public WorldSettings(bool fog, bool rain, bool clouds, bool shadows, bool sun, bool moon, bool sunLight, bool moonLight, bool freezeTime, int sunLightIntensity, int moonLightIntensity, int timeOfDay)
        {
            this.Fog = fog;
            this.Rain = rain;
            this.Clouds = clouds;
            this.Shadows = shadows;
            this.Sun = sun;
            this.Moon = moon;
            this.SunLight = sunLight;
            this.MoonLight = moonLight;
            this.FreezeTime = freezeTime;

            this.SunLightIntensity = sunLightIntensity;
            this.MoonLightIntensity = moonLightIntensity;
            this.TimeOfDay = timeOfDay;
        }
    }

    public class PlayerInformationSettings
    {
        public bool Name { get; set; }
        public bool Height { get; set; }
        public bool Distance { get; set; }
        public bool Aimline { get; set; }
        public int AimlineLength { get; set; }
        public int AimlineOpacity { get; set; }
        public int Font { get; set; }
        public int FontSize { get; set; }
        public bool Flags { get; set; }
        public bool ActiveWeapon { get; set; }
        public bool Thermal { get; set; }
        public bool NightVision { get; set; }
        public bool Gear { get; set; }
        public bool AmmoType { get; set; }
        public bool Group { get; set; }
        public bool Value { get; set; }
        public bool Health { get; set; }
        public bool Tag { get; set; }
        public int FlagsFont { get; set; }
        public int FlagsFontSize { get; set; }

        public PlayerInformationSettings(
            bool name, bool height, bool distance, bool aimline,
            int aimlineLength, int aimlineOpacity, int font, int fontSize,
            bool flags, bool activeWeapon, bool thermal, bool nightVision,
            bool gear, bool ammoType, bool group, bool value, bool health, 
            bool tag, int flagsFont, int flagsFontSize)
        {
            this.Name = name;
            this.Height = height;
            this.Distance = distance;
            this.Aimline = aimline;
            this.AimlineLength = aimlineLength;
            this.AimlineOpacity = aimlineOpacity;
            this.Font = font;
            this.FontSize = fontSize;
            this.Flags = flags;
            this.ActiveWeapon = activeWeapon;
            this.Thermal = thermal;
            this.NightVision = nightVision;
            this.Gear = gear;
            this.AmmoType = ammoType;
            this.Group = group;
            this.Value = value;
            this.Health = health;
            this.Tag = tag;
            this.FlagsFont = flagsFont;
            this.FlagsFontSize = flagsFontSize;
        }
    }

    public class AimviewSettings
    {
        public bool Enabled { get; set; }
        public int Width { get; set; }
        public int Height { get; set; }
        public int X { get; set; }
        public int Y { get; set; }
        public string TeammateID { get; set; }
        public Dictionary<string, AimviewObjectSettings> ObjectSettings { get; set; }

        public AimviewSettings(bool enabled, int width, int height, int x, int y, string teammateID, Dictionary<string, AimviewObjectSettings> objectSettings)
        {
            this.Enabled = enabled;
            this.Width = width;
            this.Height = height;
            this.X = x;
            this.Y = y;
            this.TeammateID = teammateID;
            this.ObjectSettings = objectSettings ?? Config.DefaultAimviewObjectSettings;
        }
    }

    public class AimviewObjectSettings
    {
        public bool Enabled { get; set; }
        public bool Distance { get; set; }
        public bool Name { get; set; }
        public bool Value { get; set; }
        public int PaintDistance { get; set; }
        public int TextDistance { get; set; }

        public AimviewObjectSettings(bool enabled, bool distance, bool name, bool value, int paintDistance, int textDistance)
        {
            this.Enabled = enabled;
            this.Distance = distance;
            this.Name = name;
            this.Value = value;
            this.PaintDistance = paintDistance;
            this.TextDistance = textDistance;
        }
    }

    public struct AimlineSettings
    {
        public bool Enabled;
        public int Length;
        public int Opacity;
    }

    public class Bone
    {
        private Transform _transform;
        private ulong _pointer;
        private Vector3 _position;
        private readonly object _posLock = new();
        private int _errors = 0;


        public Bone(ulong pointer)
        {
            this._pointer = pointer;
            this._transform = new Transform(pointer);
        }

        public ulong Pointer => this._pointer;

        public bool InvalidBonePtr => this._errors >= 3;

        public Vector3 Position
        {
            get
            {
                lock (this._posLock)
                    return this._position;
            }
            private set
            {
                lock (this._posLock)
                    this._position = value;
            }
        }

        public bool UpdatePosition()
        {
            try
            {
                this.Position = this._transform.GetPosition();
                
                if (this._errors > 0)
                    this._errors = 0;
                return true;
            }
            catch (Exception ex)
            {
                this._errors++;
                Program.Log($"Bone failed transform.GetPosition, attempting to get new transform");
                try
                {
                    this._transform  = new Transform(this._pointer);
                }
                catch { }

                if (this._errors >= 5)
                {
                    Program.Log("Too many bone failures, re-reading all bones.");
                    this._errors = 0;
                    return false;
                }

                return false;
            }
        }

        public void UpdateTransform(ulong pointer)
        {
            this._pointer = pointer;
            this._transform = new Transform(pointer);
        }

    }
    public class Pos3
    {
        public float X { get; }
        public float Y { get; }
        public float Z { get; }

        public Pos3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static implicit operator Pos3(Vector3 v)
        {
            return new Pos3(v.X, v.Y, v.Z);
        }
        public static implicit operator Vector3(Pos3 p)
        {
            return new Vector3(p.X, p.Y, p.Z);
        }

        public static Pos3 operator +(Pos3 a, Pos3 b)
        {
            return new Pos3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Pos3 operator +(Pos3 a, Vector3 b)
        {
            return new Pos3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Pos3 operator -(Pos3 a, Pos3 b)
        {
            return new Pos3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Pos3 operator -(Pos3 a, Vector3 b)
        {
            return new Pos3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Pos3 operator *(Pos3 a, float scalar)
        {
            return new Pos3(a.X * scalar, a.Y * scalar, a.Z * scalar);
        }

        public static Pos3 operator /(Pos3 a, float scalar)
        {
            return new Pos3(a.X / scalar, a.Y / scalar, a.Z / scalar);
        }

        public float Length()
        {
            return (float)Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        // Returns the angle between two positions in degrees.
        public static Angle2 CalcAngleDegrees(Pos3 from, Pos3 to)
        {
            Pos3 delta = from - to;
            float length = delta.Length();

            return new Angle2(
                (float)-Math.Atan2(delta.X, -delta.Z),
                (float)Math.Asin(delta.Y / length)
            );
        }

        public static Angle2 CalcAngleRadians(Pos3 from, Pos3 to)
        {
            Angle2 angle = CalcAngleDegrees(from, to);
            angle.ToRadians();
            return angle;
        }

        public override string ToString()
        {
            return $"Pos3({X}, {Y}, {Z})";
        }
    }

    public class Angle2
    {
        public float X { get; set; }
        public float Y { get; set; }

        public Angle2(float x, float y)
        {
            X = x;
            Y = y;
        }
        public Angle2(Vector2 vec2)
        {
            X = vec2.X;
            Y = vec2.Y;
        }
        
        public static Angle2 Zero => new Angle2(0, 0);

        public static implicit operator Angle2(Vector2 vec2)
        {
            return new Angle2(vec2);
        }

        public static implicit operator Vector2(Angle2 angle)
        {
            return new Vector2(angle.X, angle.Y);
        }

        public static Angle2 operator +(Angle2 angle1, Angle2 angle2)
        {
            float resultX = angle1.X + angle2.X;
            float resultY = angle1.Y + angle2.Y;
            return new Angle2(resultX, resultY);
        }

        public static Angle2 operator -(Angle2 angle1, Angle2 angle2)
        {
            float resultX = angle1.X - angle2.X;
            float resultY = angle1.Y - angle2.Y;
            return new Angle2(resultX, resultY);
        }

        public static Angle2 operator *(Angle2 angle, float scalar)
        {
            float resultX = angle.X * scalar;
            float resultY = angle.Y * scalar;
            return new Angle2(resultX, resultY);
        }
        public static Angle2 operator *(Angle2 angle, double scalar)
        {
            float resultX = angle.X * (float)scalar;
            float resultY = angle.Y * (float)scalar;
            return new Angle2(resultX, resultY);
        }

        public static Angle2 operator /(Angle2 angle, float scalar)
        {
            float resultX = angle.X / scalar;
            float resultY = angle.Y / scalar;
            return new Angle2(resultX, resultY);
        }

        public static bool operator ==(Angle2 angle1, Angle2 angle2)
        {
            if (ReferenceEquals(angle1, angle2))
                return true;
            if (angle1 is null || angle2 is null)
                return false;
            return angle1.X == angle2.X && angle1.Y == angle2.Y;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        public static bool operator !=(Angle2 angle1, Angle2 angle2)
        {
            return !(angle1 == angle2);
        }

        public override bool Equals(object obj)
        {
            if (obj is Angle2 angle)
                return this == angle;
            return false;
        }

        public void ToRadians()
        {
            X *= (float)(Math.PI / 180);
            Y *= (float)(Math.PI / 180);
        }

        public void ToDegrees()
        {
            X *= (float)(180 / Math.PI);
            Y *= (float)(180 / Math.PI);
        }

        public void NormalizeRadians()
        {
            while (X > Math.PI) X -= (float)(2 * Math.PI);
            while (X < -Math.PI) X += (float)(2 * Math.PI);
            while (Y > Math.PI) Y -= (float)(2 * Math.PI);
            while (Y < -Math.PI) Y += (float)(2 * Math.PI);
        }
        public void NormalizeDegrees()
        {
            while (X > 180.0f) X -= 360.0f;
            while (X < -180.0f) X += 360.0f;
            while (Y > 180.0f) Y -= 360.0f;
            while (Y < -180.0f) Y += 360.0f;
        }
    }
    #endregion

    #region EFT Enums
    [Flags]
    public enum MemberCategory : int
    {
        Default = 0, // Standard Account
        Developer = 1,
        UniqueId = 2, // EOD Account
        Trader = 4,
        Group = 8,
        System = 16,
        ChatModerator = 32,
        ChatModeratorWithPermamentBan = 64,
        UnitTest = 128,
        Sherpa = 256,
        Emissary = 512
    }

    /// <summary>
    /// Defines Player Unit Type (Player,PMC,Scav,etc.)
    /// </summary>
    public enum PlayerType
    {
        Default,
        LocalPlayer,
        Teammate,
        PMC,
        Scav,
        Raider,
        Rogue,
        Boss,
        PlayerScav,
        Special,
        BEAR,
        USEC,
        OfflineScav,
        SniperScav,
        BossGuard,
        BossFollower,
        FollowerOfMorana,
        Cultist,
        Zombie
    }

    public enum PlayerBones
    {
        HumanBase = 0,
        HumanPelvis = 14,
        HumanLThigh1 = 15,
        HumanLThigh2 = 16,
        HumanLCalf = 17,
        HumanLFoot = 18,
        HumanLToe = 19,
        HumanRThigh1 = 20,
        HumanRThigh2 = 21,
        HumanRCalf = 22,
        HumanRFoot = 23,
        HumanRToe = 24,
        HumanSpine1 = 29,
        HumanSpine2 = 36,
        HumanSpine3 = 37,
        HumanLCollarbone = 89,
        HumanLUpperarm = 90,
        HumanLForearm1 = 91,
        HumanLForearm2 = 92,
        HumanLForearm3 = 93,
        HumanLPalm = 94,
        HumanRCollarbone = 110,
        HumanRUpperarm = 111,
        HumanRForearm1 = 112,
        HumanRForearm2 = 113,
        HumanRForearm3 = 114,
        HumanRPalm = 115,
        HumanNeck = 132,
        HumanHead = 133
    };

    public enum HotkeyType
    {
        OnKey,
        Toggle
    }

    public enum HotkeyAction
    {
        Chams,
        ImportantLoot,
        OpticalThermal,
        Recoil,
        ShowContainers,
        ShowCorpses,
        ShowLoot,
        Thirdperson,
        ThermalVision,
        TimeScale,
        WeaponSway,
        ZoomIn,
        ZoomOut
    }
    #endregion

    #region Helpers
    public static class Helpers
    {
        /// <summary>
        /// Returns the 'type' of player based on the 'role' value.
        /// </summary>
        public static readonly Dictionary<char, string> CyrillicToLatinMap = new Dictionary<char, string>
        {
                {'А', "A"}, {'Б', "B"}, {'В', "V"}, {'Г', "G"}, {'Д', "D"},
                {'Е', "E"}, {'Ё', "E"}, {'Ж', "Zh"}, {'З', "Z"}, {'И', "I"},
                {'Й', "Y"}, {'К', "K"}, {'Л', "L"}, {'М', "M"}, {'Н', "N"},
                {'О', "O"}, {'П', "P"}, {'Р', "R"}, {'С', "S"}, {'Т', "T"},
                {'У', "U"}, {'Ф', "F"}, {'Х', "Kh"}, {'Ц', "Ts"}, {'Ч', "Ch"},
                {'Ш', "Sh"}, {'Щ', "Shch"}, {'Ъ', ""}, {'Ы', "Y"}, {'Ь', ""},
                {'Э', "E"}, {'Ю', "Yu"}, {'Я', "Ya"},
                {'а', "a"}, {'б', "b"}, {'в', "v"}, {'г', "g"}, {'д', "d"},
                {'е', "e"}, {'ё', "e"}, {'ж', "zh"}, {'з', "z"}, {'и', "i"},
                {'й', "y"}, {'к', "k"}, {'л', "l"}, {'м', "m"}, {'н', "n"},
                {'о', "o"}, {'п', "p"}, {'р', "r"}, {'с', "s"}, {'т', "t"},
                {'у', "u"}, {'ф', "f"}, {'х', "kh"}, {'ц', "ts"}, {'ч', "ch"},
                {'ш', "sh"}, {'щ', "shch"}, {'ъ', ""}, {'ы', "y"}, {'ь', ""},
                {'э', "e"}, {'ю', "yu"}, {'я', "ya"}
        };

        public static string TransliterateCyrillic(string input)
        {
            StringBuilder output = new StringBuilder();

            foreach (char c in input)
            {
                output.Append(CyrillicToLatinMap.TryGetValue(c, out var latinEquivalent) ? latinEquivalent : c.ToString());
            }

            return output.ToString();
        }
    }
    #endregion
}
