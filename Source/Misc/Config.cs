﻿using System.Text.Json.Serialization;
using System.Text.Json;

namespace eft_dma_radar
{
    public class Config
    {
        #region Json Properties
       // New properties for WEb+Aimbot
        [JsonPropertyName("aimbotFOV")]
        public int AimbotFOV { get; set; }

        [JsonPropertyName("aimbotMaxDistance")]
        public int AimbotMaxDistance { get; set; }

        [JsonPropertyName("aimbotKeybind")]
        public int AimbotKeybind { get; set; }

        [JsonPropertyName("silentAimKey")]
        public int SilentAimKey { get; set; }

        [JsonPropertyName("aimbotSmoothness")]
        public int AimbotSmoothness { get; set; }

        [JsonPropertyName("aimbotHead")]
        public bool AimbotHead { get; set; }

        [JsonPropertyName("aimbotNeck")]
        public bool AimbotNeck { get; set; }

        [JsonPropertyName("aimbotChest")]
        public bool AimbotChest { get; set; }

        [JsonPropertyName("aimbotPelvis")]
        public bool AimbotPelvis { get; set; }

        [JsonPropertyName("aimbotRightLeg")]
        public bool AimbotRightLeg { get; set; }

        [JsonPropertyName("aimbotLeftLeg")]
        public bool AimbotLeftLeg { get; set; }

        [JsonPropertyName("enableAimbot")]
        public bool EnableAimbot { get; set; }

        [JsonPropertyName("aimbotClosest")]
        public bool AimbotClosest { get; set; }
        
        [JsonPropertyName("saaimbotFOV")]
        public int SAAimbotFOV { get; set; }

        [JsonPropertyName("saaimbotMaxDistance")]
        public int SAAimbotMaxDistance { get; set; }

        [JsonPropertyName("sasilentAimKey")]
        public int SASilentAimKey { get; set; }

        [JsonPropertyName("saaimbotHead")]
        public bool SAAimbotHead { get; set; }

        [JsonPropertyName("saaimbotNeck")]
        public bool SAAimbotNeck { get; set; }

        [JsonPropertyName("saaimbotChest")]
        public bool SAAimbotChest { get; set; }

        [JsonPropertyName("saaimbotPelvis")]
        public bool SAAimbotPelvis { get; set; }

        [JsonPropertyName("saaimbotRightLeg")]
        public bool SAAimbotRightLeg { get; set; }

        [JsonPropertyName("saaimbotLeftLeg")]
        public bool SAAimbotLeftLeg { get; set; }

        [JsonPropertyName("saenableAimbot")]
        public bool SAEnableAimbot { get; set; }

        [JsonPropertyName("saaimbotClosest")]
        public bool SAAimbotClosest { get; set; }

        public bool EnableTargetScavs { get; set; } 

        [JsonPropertyName("hostname")]
        public string Hostname { get; set; }   

       // New properties for Aimbot END         
        [JsonPropertyName("aimview")]
        public bool Aimview { get; set; }

        [JsonPropertyName("aimviewFOV")]
        public float AimViewFOV { get; set; }

        [JsonPropertyName("chams")]
        public Dictionary<string, bool> Chams { get; set; }

        [JsonPropertyName("defaultZoom")]
        public int DefaultZoom { get; set; }

        [JsonPropertyName("enemyCount")]
        public bool EnemyCount { get; set; }

        [JsonPropertyName("exfilNames")]
        public bool ExfilNames { get; set; }

        [JsonPropertyName("extendedReach")]
        public bool ExtendedReach { get; set; }

        [JsonPropertyName("extendedReachDistance")]
        public float ExtendedReachDistance { get; set; }

        [JsonPropertyName("extendedReachDistancePvE")]
        public float ExtendedReachDistancePvE { get; set; }

        [JsonPropertyName("fov")]
        public int FOV { get; set; }

        [JsonPropertyName("freezeTimeOfDay")]
        public bool FreezeTimeOfDay { get; set; }

        [JsonPropertyName("globalFont")]
        public int GlobalFont { get; set; }

        [JsonPropertyName("globalFontSize")]
        public int GlobalFontSize { get; set; }

        [JsonPropertyName("hotkeys")]
        public List<Hotkey> Hotkeys { get; set; }

        [JsonPropertyName("hoverArmor")]
        public bool HoverArmor { get; set; }

        [JsonPropertyName("importantLootOnly")]
        public bool ImportantLootOnly { get; set; }

        [JsonPropertyName("infiniteStamina")]
        public bool InfiniteStamina { get; set; }

        [JsonPropertyName("instantADS")]
        public bool InstantADS { get; set; }

        [JsonPropertyName("inventoryBlur")]
        public bool InventoryBlur { get; set; }

        [JsonPropertyName("juggernaut")]
        public bool Juggernaut { get; set; }

        [JsonPropertyName("logging")]
        public bool Logging { get; set; }

        [JsonPropertyName("looseLoot")]
        public bool LooseLoot { get; set; }

        [JsonPropertyName("lootContainerDistance")]
        public int LootContainerDistance { get; set; }

        [JsonPropertyName("lootContainerSettings")]
        public Dictionary<string, bool> LootContainerSettings { get; set; }

        [JsonPropertyName("lootCorpses")]
        public bool LootCorpses { get; set; }

        [JsonPropertyName("lootItemRefresh")]
        public bool LootItemRefresh { get; set; }

        [JsonPropertyName("lootItemRefreshSettings")]
        public Dictionary<string, int> LootItemRefreshSettings { get; set; }

        [JsonPropertyName("lootItemViewer")]
        public bool LootItemViewer { get; set; }

        [JsonPropertyName("lootPing")]
        public Dictionary<string, int> LootPing { get; set; }

        [JsonPropertyName("lootThroughWalls")]
        public bool LootThroughWalls { get; set; }

        [JsonPropertyName("lootThroughWallsDistance")]
        public float LootThroughWallsDistance { get; set; }

        [JsonPropertyName("lootThroughWallsDistancePvE")]
        public float LootThroughWallsDistancePvE { get; set; }

        [JsonPropertyName("lootValue")]
        public bool LootValue { get; set; }

        [JsonPropertyName("magDrillSpeed")]
        public int MagDrillSpeed { get; set; }

        [JsonPropertyName("mainThermalSetting")]
        public ThermalSettings MainThermalSetting { get; set; }

        [JsonPropertyName("masterSwitch")]
        public bool MasterSwitch { get; set; }

        [JsonPropertyName("maxDistance")]
        public float MaxDistance { get; set; }

        [JsonPropertyName("maxSkills")]
        public Dictionary<string, bool> MaxSkills { get; set; }

        [JsonPropertyName("medInfoPanel")]
        public bool MedInfoPanel { get; set; }

        [JsonPropertyName("minCorpseValue")]
        public int MinCorpseValue { get; set; }

        [JsonPropertyName("minImportantLootValue")]
        public int MinImportantLootValue { get; set; }

        [JsonPropertyName("minLootValue")]
        public int MinLootValue { get; set; }

        [JsonPropertyName("minSubItemValue")]
        public int MinSubItemValue { get; set; }

        [JsonPropertyName("nightVision")]
        public bool NightVision { get; set; }

        [JsonPropertyName("noRecoil")]
        public bool NoRecoil { get; set; }

        [JsonPropertyName("noSway")]
        public bool NoSway { get; set; }

        [JsonPropertyName("noVisor")]
        public bool NoVisor { get; set; }

        [JsonPropertyName("noWeaponMalfunctions")]
        public bool NoWeaponMalfunctions { get; set; }

        [JsonPropertyName("opticThermalSetting")]
        public ThermalSettings OpticThermalSetting { get; set; }

        [JsonPropertyName("opticThermalVision")]
        public bool OpticThermalVision { get; set; }

        [JsonPropertyName("paintColors")]
        public Dictionary<string, PaintColor.Colors> PaintColors { get; set; }

        [JsonPropertyName("playerInformationSettings")]
        public Dictionary<string, PlayerInformationSettings> PlayerInformationSettings { get; set; }

        [JsonPropertyName("primaryTeammateAcctId")]
        public string PrimaryTeammateId { get; set; }

        [JsonPropertyName("processLoot")]
        public bool ProcessLoot { get; set; }

        [JsonPropertyName("pveMode")]
        public bool PvEMode { get; set; }

        [JsonPropertyName("questHelper")]
        public bool QuestHelper { get; set; }

        [JsonPropertyName("questItems")]
        public bool QuestItems { get; set; }

        [JsonPropertyName("questLocations")]
        public bool QuestLocations { get; set; }

        [JsonPropertyName("questLootItems")]
        public bool QuestLootItems { get; set; }

        [JsonPropertyName("questTaskRefresh")]
        public bool QuestTaskRefresh { get; set; }

        [JsonPropertyName("questTaskRefreshDelay")]
        public int QuestTaskRefreshDelay { get; set; }

        [JsonPropertyName("radarStats")]
        public bool RadarStats { get; set; }

        [JsonPropertyName("subItems")]
        public bool SubItems { get; set; }

        [JsonPropertyName("thermalVision")]
        public bool ThermalVision { get; set; }

        [JsonPropertyName("thirdperson")]
        public bool Thirdperson { get; set; }

        [JsonPropertyName("throwPowerStrength")]
        public int ThrowPowerStrength { get; set; }

        [JsonPropertyName("timeOfDay")]
        public float TimeOfDay { get; set; }

        [JsonPropertyName("timeScale")]
        public bool TimeScale { get; set; }

        [JsonPropertyName("timeScaleFactor")]
        public float TimeScaleFactor { get; set; }

        [JsonPropertyName("uiScale")]
        public int UIScale { get; set; }

        [JsonPropertyName("unknownQuestItems")]
        public bool UnknownQuestItems { get; set; }

        [JsonPropertyName("vsync")]
        public bool VSync { get; set; }

        [JsonPropertyName("zoomSensitivity")]
        public int ZoomSensitivity { get; set; }
        #endregion

        #region Json Ignore
        [JsonIgnore]
        public Dictionary<string, PaintColor.Colors> DefaultPaintColors = new Dictionary<string, PaintColor.Colors>()
        {
            // AI
            ["Boss"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 255 },
            ["BossGuard"] = new PaintColor.Colors { A = 255, R = 128, G = 0, B = 128 },
            ["BossFollower"] = new PaintColor.Colors { A = 255, R = 128, G = 0, B = 128 },
            ["Raider"] = new PaintColor.Colors { A = 255, R = 128, G = 0, B = 128 },
            ["Rogue"] = new PaintColor.Colors { A = 255, R = 128, G = 0, B = 128 },
            ["Cultist"] = new PaintColor.Colors { A = 255, R = 128, G = 0, B = 128 },
            ["Scav"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 0 },
            ["Other"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 255 },

            // Players
            ["PlayerScav"] = new PaintColor.Colors { A = 255, R = 255, G = 165, B = 0 },
            ["USEC"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 0 },
            ["BEAR"] = new PaintColor.Colors { A = 255, R = 0, G = 0, B = 255 },
            ["LocalPlayer"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 255 },
            ["Teammate"] = new PaintColor.Colors { A = 255, R = 50, G = 205, B = 50 },
            ["TeamHover"] = new PaintColor.Colors { A = 255, R = 125, G = 252, B = 50 },
            ["Special"] = new PaintColor.Colors { A = 255, R = 255, G = 105, B = 180 },

            // Exfils
            ["ExfilActiveText"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 255 },
            ["ExfilActiveIcon"] = new PaintColor.Colors { A = 255, R = 50, G = 205, B = 50 },
            ["ExfilPendingText"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 255 },
            ["ExfilPendingIcon"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 0 },
            ["ExfilClosedText"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 255 },
            ["ExfilClosedIcon"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 0 },

            // Transit
            ["TransitText"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 255 },
            ["TransitIcon"] = new PaintColor.Colors { A = 255, R = 255, G = 165, B = 0 },

            // Loot/Quests
            ["RegularLoot"] = new PaintColor.Colors { A = 255, R = 245, G = 245, B = 245 },
            ["ImportantLoot"] = new PaintColor.Colors { A = 255, R = 64, G = 224, B = 208 },
            ["QuestItem"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 128 },
            ["QuestZone"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 128 },
            ["RequiredQuestItem"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 128 },
            ["LootPing"] = new PaintColor.Colors { A = 255, R = 255, G = 255, B = 0 },

            // Game World
            ["Grenades"] = new PaintColor.Colors { A = 255, R = 255, G = 69, B = 0 },
            ["Tripwires"] = new PaintColor.Colors { A = 255, R = 255, G = 69, B = 0 },
            ["DeathMarker"] = new PaintColor.Colors { A = 255, R = 0, G = 0, B = 0 },

            // Other
            ["TextOutline"] = new PaintColor.Colors { A = 255, R = 0, G = 0, B = 0 },
            ["Chams"] = new PaintColor.Colors { A = 255, R = 255, G = 0, B = 0 },
            ["Primary"] = new PaintColor.Colors { A = 255, R = 80, G = 80, B = 80 },
            ["PrimaryDark"] = new PaintColor.Colors { A = 255, R = 50, G = 50, B = 50 },
            ["PrimaryLight"] = new PaintColor.Colors { A = 255, R = 130, G = 130, B = 130 },
            ["Accent"] = new PaintColor.Colors { A = 255, R = 255, G = 128, B = 0 },

            // Event/Temporary
            ["FollowerOfMorana"] = new PaintColor.Colors { A = 255, R = 128, G = 0, B = 128 },
            ["Zombie"] = new PaintColor.Colors { A = 255, R = 190, G = 128, B = 0 }
        };

        [JsonIgnore]
        public Dictionary<string, int> DefaultAutoRefreshSettings = new Dictionary<string, int>()
        {
            ["Customs"] = 30,
            ["Factory"] = 30,
            ["Ground Zero"] = 30,
            ["Interchange"] = 30,
            ["Lighthouse"] = 30,
            ["Reserve"] = 30,
            ["Shoreline"] = 30,
            ["Streets of Tarkov"] = 30,
            ["The Lab"] = 30,
            ["Woods"] = 30
        };

        [JsonIgnore]
        public Dictionary<string, bool> DefaultChamsSettings = new Dictionary<string, bool>()
        {
            ["AlternateMethod"] = false,
            ["Bosses"] = false,
            ["Corpses"] = false,
            ["Cultists"] = false,
            ["Enabled"] = false,
            ["PlayerScavs"] = false,
            ["PMCs"] = false,
            ["RevertOnClose"] = false,
            ["Rogues"] = false,
            ["Event"] = false,
            ["Scavs"] = false,
            ["Teammates"] = false
        };

        [JsonIgnore]
        public Dictionary<string, bool> DefaultContainerSettings = new Dictionary<string, bool>()
        {
            ["Enabled"] = false,
            ["Bank cash register"] = false,
            ["Bank safe"] = false,
            ["Buried barrel cache"] = false,
            ["Cash register"] = false,
            ["Civilian body"] = false,
            ["Dead Scav"] = false,
            ["Drawer"] = false,
            ["Duffle bag"] = false,
            ["Grenade box"] = false,
            ["Ground cache"] = false,
            ["Jacket"] = false,
            ["Lab technician body"] = false,
            ["Medbag SMU06"] = false,
            ["Medcase"] = false,
            ["Medical supply crate"] = false,
            ["PC block"] = false,
            ["PMC body"] = false,
            ["Ration supply crate"] = false,
            ["Safe"] = false,
            ["Scav body"] = false,
            ["Shturman's Stash"] = false,
            ["Technical supply crate"] = false,
            ["Toolbox"] = false,
            ["Weapon box"] = false,
            ["Wooden ammo box"] = false,
            ["Wooden crate"] = false
        };

        [JsonIgnore]
        public Dictionary<string, int> DefaultLootPingSettings = new Dictionary<string, int>()
        {
            ["AnimationSpeed"] = 1000,
            ["Radius"] = 20,
            ["Repetition"] = 1
        };

        [JsonIgnore]
        public Dictionary<string, bool> DefaultMaxSkillsSettings = new Dictionary<string, bool>()
        {
            ["Endurance"] = false,
            ["Strength"] = false,
            ["Vitality"] = false,
            ["Health"] = false,
            ["Stress Resistance"] = false,
            ["Metabolism"] = false,
            ["Immunity"] = false,
            ["Perception"] = false,
            ["Intellect"] = false,
            ["Attention"] = false,
            ["Covert Movement"] = false,
            ["Throwables"] = false,
            ["Surgery"] = false,
            ["Search"] = false,
            ["Mag Drills"] = false,
            ["Light Vests"] = false,
            ["Heavy Vests"] = false,
        };

        [JsonIgnore]
        public Dictionary<string, PlayerInformationSettings> DefaultPlayerInformationSettings = new Dictionary<string, PlayerInformationSettings>()
        {
            ["Boss"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["BossFollower"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["BossGuard"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Cultist"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["LocalPlayer"] = new PlayerInformationSettings(true, true, true, true, 500, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["PMC"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["PlayerScav"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Raider"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Rogue"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Scav"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Special"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Teammate"] = new PlayerInformationSettings(true, true, true, true, 500, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
            ["Zombie"] = new PlayerInformationSettings(true, true, true, true, 15, 255, 0, 13, false, false, false, false, false, false, false, false, false, false, 0, 13),
        };

        [JsonIgnore]
        public List<LootFilterManager.Filter> Filters
        {
            get => LootFilterManager.Filters;
        }

        [JsonIgnore]
        private static readonly JsonSerializerOptions _jsonOptions = new JsonSerializerOptions()
        {
            WriteIndented = true
        };

        [JsonIgnore]
        private static readonly object _lock = new();

        [JsonIgnore]
        public LootFilterManager LootFilterManager
        {
            get => Program.LootFilterManager;
        }

        [JsonIgnore]
        public ParallelOptions ParallelOptions
        {
            get; set;
        }

        [JsonIgnore]
        public List<Watchlist.Profile> Profiles
        {
            get => Watchlist.Profiles;
        }

        [JsonIgnore]
        private const string SettingsDirectory = "Configuration\\";

        [JsonIgnore]
        public Watchlist Watchlist
        {
            get => Program.Watchlist;
        }
        #endregion

        public Config()
        {
            AimViewFOV = 30;
            Aimview = false;
            Chams = DefaultChamsSettings;
            DefaultZoom = 100;
            EnemyCount = false;
            ExfilNames = false;
            ExtendedReach = false;
            ExtendedReachDistance = 2f;
            ExtendedReachDistancePvE = 2f;
            FOV = 75;
            FreezeTimeOfDay = false;
            GlobalFont = 0;
            GlobalFontSize = 13;
            Hotkeys = new List<Hotkey>();
            HoverArmor = false;
            ImportantLootOnly = false;
            InfiniteStamina = false;
            InstantADS = false;
            InventoryBlur = false;
            Juggernaut = false;
            Logging = false;
            LooseLoot = true;
            LootContainerDistance = 300;
            LootContainerSettings = DefaultContainerSettings;
            LootCorpses = false;
            LootItemRefresh = false;
            LootItemRefreshSettings = DefaultAutoRefreshSettings;
            LootItemViewer = false;
            LootPing = DefaultLootPingSettings;
            LootThroughWalls = false;
            LootThroughWallsDistance = 2f;
            LootThroughWallsDistancePvE = 2f;
            LootValue = false;
            MagDrillSpeed = 1;
            MainThermalSetting = new ThermalSettings(1f, 0.0011f, -0.1f, 0);
            MasterSwitch = false;
            MaxDistance = 325;
            MaxSkills = DefaultMaxSkillsSettings;
            MedInfoPanel = false;
            MinCorpseValue = 100000;
            MinImportantLootValue = 300000;
            MinLootValue = 90000;
            MinSubItemValue = 15000;
            NightVision = false;
            NoRecoil = false;
            NoSway = false;
            NoVisor = false;
            NoWeaponMalfunctions = false;
            OpticThermalSetting = new ThermalSettings(1f, 0.0011f, -0.1f, 0);
            OpticThermalVision = false;
            PaintColors = DefaultPaintColors;
            ParallelOptions = new ParallelOptions { MaxDegreeOfParallelism = 2 };
            PlayerInformationSettings = DefaultPlayerInformationSettings;
            PrimaryTeammateId = null;
            ProcessLoot = true;
            PvEMode = false;
            QuestHelper = false;
            QuestItems = false;
            QuestLocations = false;
            QuestLootItems = false;
            QuestTaskRefresh = false;
            QuestTaskRefreshDelay = 15;
            RadarStats = false;
            SubItems = false;
            ThermalVision = false;
            Thirdperson = false;
            ThrowPowerStrength = 1;
            TimeOfDay = 12f;
            TimeScale = false;
            TimeScaleFactor = 1.8f;
            UIScale = 100;
            UnknownQuestItems = false;
            VSync = true;
            ZoomSensitivity = 25;
            //Web+Aimbot
            AimbotFOV = 30;
            AimbotMaxDistance = 200;
            AimbotKeybind = 0x05; // Example: Mouse button 5
            SilentAimKey = 0x06;
            AimbotSmoothness = 100;
            AimbotHead = false;  // Enable aiming at the head
            AimbotNeck = false; // Disable aiming at the neck
            AimbotChest = false; // Enable aiming at the chest
            AimbotPelvis = false; // Disable aiming at the pelvis
            AimbotRightLeg = false; // Disable aiming at the right leg
            AimbotLeftLeg = false; // Disable aiming at the left leg
            EnableAimbot = false; // Enable the aimbot feature
            AimbotClosest = false;
            SAAimbotFOV = 30;
            SAAimbotMaxDistance = 200;
            SASilentAimKey = 0x06;
            SAAimbotHead = true;  // Enable aiming at the head
            SAAimbotNeck = false; // Disable aiming at the neck
            SAAimbotChest = false; // Enable aiming at the chest
            SAAimbotPelvis = false; // Disable aiming at the pelvis
            SAAimbotRightLeg = false; // Disable aiming at the right leg
            SAAimbotLeftLeg = false; // Disable aiming at the left leg
            SAEnableAimbot = true; // Enable the aimbot feature
            SAAimbotClosest = false;
            Hostname = "localhost";
        }

        /// <summary>
        /// Attempt to load Config.json
        /// </summary>
        /// <param name="config">'Config' instance to populate.</param>
        /// <returns></returns>
        public static bool TryLoadConfig(out Config config)
        {
            lock (_lock)
            {
                if (!Directory.Exists(SettingsDirectory))
                    Directory.CreateDirectory(SettingsDirectory);

                try
                {
                    if (!File.Exists($"{SettingsDirectory}Settings.json"))
                        throw new FileNotFoundException("Settings.json does not exist!");

                    var json = File.ReadAllText($"{SettingsDirectory}Settings.json");

                    config = JsonSerializer.Deserialize<Config>(json);
                    return true;
                }
                catch (Exception ex)
                {
                    Program.Log($"TryLoadConfig - {ex.Message}\n{ex.StackTrace}");
                    config = null;
                    return false;
                }
            }
        }
        /// <summary>
        /// Save to Config.json
        /// </summary>
        /// <param name="config">'Config' instance</param>
        public static void SaveConfig(Config config)
        {
            lock (_lock)
            {
                if (!Directory.Exists(SettingsDirectory))
                    Directory.CreateDirectory(SettingsDirectory);

                var json = JsonSerializer.Serialize<Config>(config, _jsonOptions);
                File.WriteAllText($"{SettingsDirectory}Settings.json", json);
            }
        }

        public bool GetConfigValue(string actionName)
        {
            return actionName switch
            {
                "Chams" => this.Chams["Enabled"],
                "Important Loot" => this.ImportantLootOnly,
                "No Recoil" => this.NoRecoil,
                "No Sway" => this.NoSway,
                "Optical Thermal" => this.OpticThermalVision,
                "Show Containers" => this.LootContainerSettings["Enabled"],
                "Show Corpses" => this.LootCorpses,
                "Show Loot" => this.LooseLoot,
                "Thirdperson" => this.Thirdperson,
                "Thermal Vision" => this.ThermalVision,
                "Timescale" => this.TimeScale,
                _ => false
            };
        }
    }
}
