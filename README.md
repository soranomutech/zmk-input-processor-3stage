# zmk-input-processor-3stage

A ZMK input processor module that provides **3-stage velocity-based pointing acceleration**.

## Acceleration Curve

```
factor
max  |                                          ___/
     |                                     ___/
mid  |          ___________________________/
     |     ___/
min  |____/
     +--------+-----------+----------------+------> speed (counts/sec)
     0    speed-low   speed-high       speed-max
       decel zone   flat zone       accel zone
```

| Zone | Speed Range | Factor | Use Case |
|------|-------------|--------|----------|
| Deceleration | 0 → speed-low | min-factor → mid-factor | Precision aiming |
| Flat | speed-low → speed-high | mid-factor (constant) | Normal use |
| Acceleration | speed-high → speed-max | mid-factor → max-factor | Fast cursor travel |

## Setup

### 1. Add to `west.yml`

```yaml
manifest:
  remotes:
    - name: soranomutech
      url-base: https://github.com/soranomutech
  projects:
    - name: zmk-input-processor-3stage
      remote: soranomutech
      revision: main
```

### 2. Enable in `.conf`

```ini
CONFIG_ZMK_INPUT_PROCESSOR_3STAGE=y
```

### 3. Configure in overlay

```dts
#include <behaviors/input_processor_3stage.dtsi>

&pointer_accel_3s {
    status = "okay";
    input-type = <INPUT_EV_REL>;
    min-factor = <560>;           // 0.56x at slow speeds
    mid-factor = <1000>;          // 1.0x normal speed
    max-factor = <3000>;          // 3.0x at fast speeds
    speed-low = <1200>;           // decel → flat boundary
    speed-high = <1800>;          // flat → accel boundary
    speed-max = <6000>;           // max-factor reached here
    deceleration-exponent = <1>;  // linear decel curve
    acceleration-exponent = <2>;  // quadratic accel curve
    track-remainders;             // accumulate fractional movement
};
```

### 4. Add to input processor chain

```dts
input-processors = <&pointer_accel_3s>, <&other_processors ...>;
```

## Properties

| Property | Required | Default | Description |
|----------|----------|---------|-------------|
| `input-type` | yes | - | Input event type (`INPUT_EV_REL`) |
| `min-factor` | yes | - | Factor at speed=0 (x1000 scale) |
| `mid-factor` | yes | - | Factor in flat zone (x1000 scale) |
| `max-factor` | yes | - | Factor at speed-max (x1000 scale) |
| `speed-low` | yes | - | End of decel zone (counts/sec). Set 0 to disable. |
| `speed-high` | yes | - | Start of accel zone (counts/sec) |
| `speed-max` | yes | - | Speed at which max-factor is reached |
| `deceleration-exponent` | no | 1 | Decel curve shape (1=linear, 2=quadratic) |
| `acceleration-exponent` | no | 2 | Accel curve shape (1=linear, 2=quadratic) |
| `track-remainders` | no | false | Accumulate fractional movements |

## Tips

- Set `speed-low = <0>` to disable deceleration (2-stage mode)
- Set `min-factor = mid-factor` for no deceleration effect
- Set `mid-factor = max-factor` for deceleration-only mode
- All factors use x1000 scale: 500 = 0.5x, 1000 = 1.0x, 3000 = 3.0x

## License

MIT
