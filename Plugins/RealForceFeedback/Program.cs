using ETS2LA.Shared;
using ETS2LA.Logging;
using ETS2LA.Telemetry;
using ETS2LA.Backend.Events;
using ETS2LA.UI.Notifications;
using ETS2LA.Settings;
using SharpDX.DirectInput;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;

// FF listens to "ForceFeedback.Output" bus event for float values between -1.0 and 1.0.
// If you want to use this plugin then make sure you send those events in addition to normal output.
// Example: _bus.Publish<float>("ForceFeedback.Output", steeringValue);

namespace ForceFeedback
{
    [Serializable]
    public class FfbSettings
    {
        // PID gains
        public float Kp { get; set; } = 1.0f;
        public float Ki { get; set; } = 0.05f;
        public float Kd { get; set; } = 0.5f;
        public float MaxIntegral { get; set; } = 0.5f;

        // Force output limits
        public float MaxPidForce { get; set; } = 0.15f;
        public float MaxTotalForce { get; set; } = 1.0f;

        // Smoothing (0.0 = no smoothing, higher = more smoothing)
        public float Smoothing { get; set; } = 0.3f;

        // Engine vibration
        public bool EnableEngineVibration { get; set; } = true;
        public float EngineVibrationScale { get; set; } = 1.0f;

        // Road bump effects
        public bool EnableBumpEffects { get; set; } = true;
        public float BumpSensitivity { get; set; } = 0.1f;
        public float MaxBumpForce { get; set; } = 0.3f;
    }

    public class WheelDevice
    {
        public required Joystick Joystick;
        public required DeviceInstance DeviceInfo;
        public EffectInfo? ConstantForceInfo;
        public Effect? ConstantForceEffect;
        public Effect? EngineEffect;
        public bool IsInitialized = false;
        public bool IsLowPower = true;

        private readonly object _lock = new object();
        private float _last = 0;
        private float _lastError = 0;
        private float _integralError = 0;
        private FfbSettings _settings;

        // Cached effect parameters to reduce GC pressure (reused every tick)
        private EffectParameters? _cachedEffectParams;
        private ConstantForce _cachedConstantForce = new ConstantForce();
        private EffectParameters? _cachedEngineParams;
        private PeriodicForce _cachedPeriodic = new PeriodicForce();

        public WheelDevice(FfbSettings settings)
        {
            _settings = settings;
        }

        public void ApplySettings(FfbSettings settings)
        {
            _settings = settings;
        }


        public void UpdateToAngle(float targetAngle)
        {
            lock (_lock)
            {
                if (!IsInitialized || ConstantForceEffect == null) return;

                try
                {
                    var state = Joystick.GetCurrentState();
                    // X axis is typically the steering wheel, normalized to -1.0 to 1.0
                    float currentAngle = (32767.5f - state.X) / 32767.5f;

                    float error = targetAngle - currentAngle;
                    
                    // Integral term 
                    _integralError += error;
                    _integralError = Math.Clamp(_integralError, -_settings.MaxIntegral, _settings.MaxIntegral);
                    if (Math.Sign(error) != Math.Sign(_lastError) || Math.Abs(error) < 0.02f)
                    {
                        _integralError *= 0.5f;
                    }
                    
                    // PID calculation
                    float derivative = error - _lastError;
                    _lastError = error;
                    
                    float force = error * _settings.Kp + _integralError * _settings.Ki + derivative * _settings.Kd;
                    force = Math.Clamp(force, -_settings.MaxPidForce, _settings.MaxPidForce);
                    force += _bumpForce;
                    
                    force = Math.Clamp(force, -_settings.MaxTotalForce, _settings.MaxTotalForce);
                    force = _last * _settings.Smoothing + force * (1.0f - _settings.Smoothing);
                    _last = force;

                    /* 
                    // Boost weak forces to overcome static friction on some wheels
                    if (IsLowPower && Math.Abs(force) > 0.03f && Math.Abs(force) < 0.15f)
                    {
                        force = Math.Sign(force) * 0.15f;
                    }
                    */

                    // DirectInput force is in the range -10000 to 10000
                    int diForce = (int)(force * 10000);

                    // Initialize cached params on first use
                    if (_cachedEffectParams == null)
                    {
                        _cachedEffectParams = new EffectParameters
                        {
                            Flags = EffectFlags.Cartesian | EffectFlags.ObjectOffsets,
                            Duration = int.MaxValue,
                            Gain = 10000,
                            SamplePeriod = 0,
                            TriggerButton = -1,
                            TriggerRepeatInterval = 0,
                            StartDelay = 0
                        };
                        _cachedEffectParams.SetAxes(new int[] { 0 }, new int[] { 0 });
                    }

                    // Update only the magnitude (reuse cached objects)
                    _cachedConstantForce.Magnitude = diForce;
                    _cachedEffectParams.Parameters = _cachedConstantForce;
                    ConstantForceEffect.SetParameters(_cachedEffectParams, EffectParameterFlags.TypeSpecificParameters);
                }
                catch (Exception ex)
                {
                    Logger.Warn($"Error updating force feedback: {ex.Message}");
                }
            }
        }

        public void InitializeEffect()
        {
            try
            {
                // Find constant force effect
                var effects = Joystick.GetEffects();
                foreach (var effectInfo in effects)
                {
                    if (effectInfo.Guid == EffectGuid.ConstantForce)
                    {
                        ConstantForceInfo = effectInfo;
                        break;
                    }
                }

                if (ConstantForceInfo == null)
                {
                    Logger.Warn($"No constant force effect found for {DeviceInfo.InstanceName}");
                    return;
                }

                // Create the effect parameters
                var constantForce = new ConstantForce { Magnitude = 0 };

                var effectParams = new EffectParameters
                {
                    Flags = EffectFlags.Cartesian | EffectFlags.ObjectOffsets,
                    Duration = int.MaxValue,
                    SamplePeriod = 0,
                    Gain = 10000,
                    TriggerButton = -1,
                    TriggerRepeatInterval = 0,
                    StartDelay = 0
                };

                // Set axes - typically just X for steering
                effectParams.SetAxes(new int[] { 0 }, new int[] { 0 });
                effectParams.Parameters = constantForce;

                // Create and start the effect
                ConstantForceEffect = new Effect(Joystick, EffectGuid.ConstantForce, effectParams);
                ConstantForceEffect.Start();

                IsInitialized = true;
                Logger.Success($"Initialized force feedback for {DeviceInfo.InstanceName}");
            }
            catch (Exception ex)
            {
                Logger.Error($"Failed to initialize force feedback for {DeviceInfo.InstanceName}: {ex.Message}");
                IsInitialized = false;
            }
        }

        public void InitializeEngineEffect()
        {
            try
            {
                var effects = Joystick.GetEffects();
                if (!effects.Any(x => x.Guid == EffectGuid.Sine))
                {
                    Logger.Warn($"Sine wave effect not supported by {DeviceInfo.InstanceName} - engine vibration disabled");
                    return;
                }

                var periodic = new PeriodicForce
                {
                    Magnitude = 0,
                    Period = 100000, 
                    Phase = 0
                };

                _cachedEngineParams = new EffectParameters
                {
                    Flags = EffectFlags.Cartesian | EffectFlags.ObjectOffsets,
                    Duration = int.MaxValue,
                    SamplePeriod = 0,
                    Gain = 10000,
                    TriggerButton = -1,
                    TriggerRepeatInterval = 0,
                    StartDelay = 0
                };
                _cachedEngineParams.SetAxes(new int[] { 0 }, new int[] { 0 });
                _cachedEngineParams.Parameters = periodic;

                EngineEffect = new Effect(Joystick, EffectGuid.Sine, _cachedEngineParams);
                EngineEffect.Start();
                Logger.Info($"Initialized engine vibration for {DeviceInfo.InstanceName}");
            }
            catch (Exception ex)
            {
                Logger.Warn($"Failed to initialize engine vibration: {ex.Message}");
            }
        }

        private Vector3 _lastAccel = Vector3.Zero;
        private float _bumpForce = 0;

        public void UpdateTelemetry(float rpm, float speed, Vector3 acceleration)
        {
            lock (_lock)
            {
                if (_settings.EnableBumpEffects)
                {
                    float currentAccelY = acceleration.Y;
                    float distY = Math.Abs(currentAccelY - _lastAccel.Y);
                    if (distY > 0.05f) 
                    {
                        float bumpMagnitude = Math.Clamp(distY * _settings.BumpSensitivity, 0, 0.2f);
                        if ((DateTime.Now.Ticks % 2) == 0) bumpMagnitude = -bumpMagnitude;
                        _bumpForce += bumpMagnitude;
                        _bumpForce = Math.Clamp(_bumpForce, -_settings.MaxBumpForce, _settings.MaxBumpForce);
                    }
                }
                
                _bumpForce *= 0.85f;
                _lastAccel = acceleration;
                if (!_settings.EnableEngineVibration || EngineEffect == null || _cachedEngineParams == null) return;
                try
                {
                    int magnitude = 0;
                    int period = 100000;

                    if (rpm > 100)
                    {
                        period = (int)Math.Clamp(60000000.0f / rpm, 1000, 200000);
                        float vibrationScale = 0.05f;
                        if (rpm < 700) 
                            vibrationScale += 0.15f * (1.0f - (rpm - 300f)/400f);
                        if (rpm > 1500)
                            vibrationScale += 0.1f * ((rpm - 1500f)/1000f);

                        magnitude = (int)(Math.Clamp(vibrationScale, 0, 0.4f) * 10000 * _settings.EngineVibrationScale);
                    }

                    _cachedPeriodic.Magnitude = magnitude;
                    _cachedPeriodic.Period = period;
                    _cachedEngineParams.Parameters = _cachedPeriodic;

                    EngineEffect.SetParameters(_cachedEngineParams, EffectParameterFlags.TypeSpecificParameters);
                }
                catch { }
            }
        }

        public void StopEffect()
        {
            lock (_lock)
            {
                try
                {
                    // Set force to zero before stopping to release the wheel smoothly
                    if (IsInitialized && ConstantForceEffect != null && _cachedEffectParams != null)
                    {
                        _cachedConstantForce.Magnitude = 0;
                        _cachedEffectParams.Parameters = _cachedConstantForce;
                        ConstantForceEffect.SetParameters(_cachedEffectParams, EffectParameterFlags.TypeSpecificParameters);
                    }
                    
                    IsInitialized = false; // Set first to prevent UpdateToAngle from using effect
                    ConstantForceEffect?.Stop();
                    ConstantForceEffect?.Dispose();
                    ConstantForceEffect = null;

                    EngineEffect?.Stop();
                    EngineEffect?.Dispose();
                    EngineEffect = null;
                }
                catch (Exception) { }
            }
        }

        public void Dispose()
        {
            StopEffect();
            try
            {
                if (Joystick != null)
                {
                    // Re-enable auto-center spring to center the wheel and unlock it
                    try 
                    { 
                        Joystick.Properties.AutoCenter = true;
                        Logger.Info($"Re-enabled auto-center for {DeviceInfo.InstanceName}");
                    } 
                    catch { }
                }

                Joystick?.Unacquire();
                Joystick?.Dispose();
                Logger.Info($"Disposed wheel: {DeviceInfo.InstanceName}");
            }
            catch (Exception) { }
        }
    }

    public class ForceFeedback : Plugin
    {
        [DllImport("user32.dll")]
        private static extern IntPtr GetForegroundWindow();

        [DllImport("user32.dll")]
        private static extern IntPtr GetDesktopWindow();

        public override float TickRate => 60f;
        public override PluginInformation Info => new PluginInformation
        {
            Name = "Real Force Feedback",
            Description = "Recreates force feedback effects from ETS2/ATS.",
            AuthorName = "Zkhan4509",
        };

        private DirectInput? _directInput;
        private List<WheelDevice> _wheels = new List<WheelDevice>();
        public float TargetAngle = 0;
        private float _lastRpm = 0;
        private float _lastSpeed = 0;
        private Vector3 _lastAcceleration = Vector3.Zero;
        private IntPtr _windowHandle = IntPtr.Zero;
        private int _scanRetryCount = 0;
        private const int MaxScanRetries = 300;
        private bool _isScanning = false;
        private readonly object _scanLock = new object();

        private SettingsHandler? _settingsHandler;
        private FfbSettings _settings = new FfbSettings();
        private const string SettingsFilename = "realffb_settings.json";

        [DllImport("kernel32.dll")]
        private static extern IntPtr GetConsoleWindow();

        private delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

        [DllImport("user32.dll")]
        private static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);

        [DllImport("user32.dll")]
        private static extern bool IsWindowVisible(IntPtr hWnd);

        [DllImport("user32.dll")]
        private static extern int GetWindowThreadProcessId(IntPtr hWnd, out int lpdwProcessId);

        private IntPtr GetWindowHandle()
        {
            var currentProcessId = Process.GetCurrentProcess().Id;
            var consoleHandle = GetConsoleWindow();
            IntPtr found = IntPtr.Zero;

            EnumWindows((hWnd, _) =>
            {
                if (!IsWindowVisible(hWnd)) return true;
                if (hWnd == consoleHandle) return true;

                GetWindowThreadProcessId(hWnd, out int pid);
                if (pid == currentProcessId)
                {
                    found = hWnd;
                    return false;
                }
                return true;
            }, IntPtr.Zero);

            if (found != IntPtr.Zero)
            {
                Logger.Info($"Using app window handle: {found}");
                return found;
            }

            Logger.Warn("No valid window handle found for force feedback. Will retry on next scan.");
            return IntPtr.Zero;
        }

        private void ScanForWheels()
        {
            // Prevent concurrent scanning
            lock (_scanLock)
            {
                if (_isScanning) return;
                _isScanning = true;
            }

            try
            {
                if (_directInput == null)
                {
                    Logger.Error("DirectInput is null, cannot scan for wheels");
                    return;
                }

                // First, list ALL devices to see what's available
                var allDevices = _directInput.GetDevices(DeviceClass.GameControl, DeviceEnumerationFlags.AllDevices);
                Logger.Info($"Found {allDevices.Count} total game control devices.");
                foreach (var dev in allDevices)
                {
                    Logger.Info($"  Device: {dev.InstanceName} ({dev.ProductName}) - Type: {dev.Type}");
                }

                // Get all game controllers and driving devices with force feedback
                var devices = _directInput.GetDevices(DeviceClass.GameControl, DeviceEnumerationFlags.ForceFeedback);

                Logger.Info($"Found {devices.Count} force feedback capable devices.");

                foreach (var deviceInstance in devices)
                {
                    bool alreadyAdded = _wheels.Any(w => w.DeviceInfo.InstanceGuid == deviceInstance.InstanceGuid);
                    if (alreadyAdded) continue;

                    Joystick? joystick = null;
                    try
                    {
                        joystick = new Joystick(_directInput, deviceInstance.InstanceGuid);
                        
                        // Get window handle for exclusive access (required for force feedback on some devices)
                        if (_windowHandle == IntPtr.Zero)
                        {
                            _windowHandle = GetWindowHandle();
                            if (_windowHandle == IntPtr.Zero)
                            {
                                Logger.Warn("Cannot initialize force feedback: no valid window handle available.");
                                joystick.Dispose();
                                continue;
                            }
                            Logger.Info($"Using window handle: {_windowHandle}");
                        }
                        
                        // Exclusive access is required for force feedback
                        try
                        {
                            joystick.SetCooperativeLevel(_windowHandle, CooperativeLevel.Background | CooperativeLevel.Exclusive);
                        }
                        catch (SharpDX.SharpDXException sdxEx)
                        {
                            Logger.Warn($"Exclusive access failed (0x{sdxEx.HResult:X8}), skipping device until retry");
                            joystick.Dispose();
                            joystick = null;
                            _windowHandle = IntPtr.Zero;
                            continue;
                        }
                        
                        joystick.Acquire();

                        // Disable auto-center spring
                        try
                        {
                            joystick.Properties.AutoCenter = false;
                        }
                        catch (Exception) { /* Some devices don't support this */ }

                        var wheel = new WheelDevice(_settings)
                        {
                            Joystick = joystick,
                            DeviceInfo = deviceInstance
                        };

                        wheel.InitializeEffect();
                        wheel.InitializeEngineEffect();
                        _wheels.Add(wheel);
                        joystick = null;

                        Logger.Info($"Added wheel: {deviceInstance.InstanceName} ({deviceInstance.ProductName})");
                    }
                    catch (Exception ex)
                    {
                        Logger.Warn($"Could not add device {deviceInstance.InstanceName}: {ex.Message}");
                    }
                    finally
                    {
                        joystick?.Dispose();
                    }
                }
            }
            catch (Exception ex)
            {
                Logger.Error($"Error scanning for wheels: {ex.Message}");
            }
            finally
            {
                lock (_scanLock)
                {
                    _isScanning = false;
                }
            }
        }

        public override void Init()
        {
            base.Init();
            Logger.Info("ForceFeedback plugin Init() called");

            try
            {
                _directInput = new DirectInput();
                Logger.Info("DirectInput created successfully");
                ScanForWheels();
            }
            catch (Exception ex)
            {
                Logger.Error($"Failed to initialize DirectInput: {ex.Message}");
                Logger.Error(ex.ToString());
            }
        }

        public override void OnEnable()
        {
            base.OnEnable();
            Logger.Info("ForceFeedback plugin OnEnable() called");

            _settingsHandler = new SettingsHandler();
            _settings = _settingsHandler.Load<FfbSettings>(SettingsFilename);
            _settingsHandler.RegisterListener<FfbSettings>(SettingsFilename, OnSettingsChanged);

            Events.Current.Subscribe<float>("ForceFeedback.Output", OnControlEvent);
            Events.Current.Subscribe<GameTelemetryData>(GameTelemetry.Current.EventString, OnTelemetry);

            if (_directInput == null)
            {
                try
                {
                    _directInput = new DirectInput();
                    Logger.Info("DirectInput re-created successfully");
                }
                catch (Exception ex)
                {
                    Logger.Error($"Failed to re-create DirectInput: {ex.Message}");
                    return;
                }
            }

            ScanForWheels();
        }

        public override void OnDisable()
        {
            base.OnDisable();
            Events.Current.Unsubscribe<float>("ForceFeedback.Output", OnControlEvent);
            Events.Current.Unsubscribe<GameTelemetryData>(GameTelemetry.Current.EventString, OnTelemetry);
            _settingsHandler?.UnregisterListener<FfbSettings>(SettingsFilename, OnSettingsChanged);
            _settingsHandler?.Dispose();
            _settingsHandler = null;
            NotificationHandler.Current.CloseNotification("ForceFeedback.Debug");
            DisposeResources();
        }

        public void OnControlEvent(float steering)
        {
            TargetAngle = steering;
        }

        public void OnTelemetry(GameTelemetryData data)
        {
            _lastRpm = data.truckFloat.engineRpm;
            _lastSpeed = data.truckFloat.speed;
            _lastAcceleration = data.truckVector.acceleration;
            TargetAngle = data.truckFloat.gameSteer;
        }

        private void OnSettingsChanged(FfbSettings data)
        {
            _settings = data;
            foreach (var wheel in _wheels)
            {
                wheel.ApplySettings(data);
            }
            Logger.Info("RealFFB settings updated");
        }

        public override void Tick()
        {
            try
            {
                if (_wheels.Count == 0 && _scanRetryCount < MaxScanRetries && _directInput != null && !_isScanning)
                {
                    _scanRetryCount++;
                    _windowHandle = IntPtr.Zero;
                    Logger.Info($"Retrying wheel scan (attempt {_scanRetryCount}/{MaxScanRetries})...");
                    ScanForWheels();
                }

                // Copy to local list to avoid modification during iteration
                var wheels = _wheels.ToList();
                foreach (var wheel in wheels)
                {
                    wheel.UpdateToAngle(TargetAngle);
                    wheel.UpdateTelemetry(_lastRpm, _lastSpeed, _lastAcceleration);
                }
            }
            catch (Exception ex)
            {
                Logger.Error($"Error in ForceFeedback Tick: {ex.Message}");
            }
        }

        private void DisposeResources()
        {
            Logger.Info("Disposing force feedback resources - centering and unlocking wheels...");
            foreach (var wheel in _wheels)
            {
                wheel.Dispose();
            }
            _wheels.Clear();
            _directInput?.Dispose();
            _directInput = null;
            _windowHandle = IntPtr.Zero;
            _scanRetryCount = 0;
            Logger.Info("Force feedback resources disposed.");
        }
    }
}