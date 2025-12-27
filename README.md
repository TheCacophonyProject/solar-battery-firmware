# Notes

## Set input source

If we disable input source
Remove automatic source detection
Need to enable MPPT

## Protection

This table shows the different states, starting with the default (healthy), and then the different states that we need to add protection for (over/under voltage/temp...) and it shows what type of protection is needed for each state.

| State                 | CHG       | DSG       | Charging Limit (mA)| Balancing|
|---------------        |-----------|-----------|--------------------|-----     |
| Default (healthy)     | ENABLE    | ENABLE    | 2000               | ENABLE   |
| Cell Over Voltage     | DISABLE   |           |                    |          |
| Cell Under Voltage    |           | DISABLE   |                    |          |
| Pack Over Voltage     | DISABLE   |           |                    |          |
| Pack Warm             |           |           | 500                |          |
| Pack Cool             |           |           | 500                |          |
| Pack Too Cold         | DISABLE   | DISABLE   |                    | DISABLE  |
| Pack Too Hot          | DISABLE   | DISABLE   |                    | DISABLE  |

Other states that we will want to test we can protect against and recover from.

- Input over voltage: At some point this will just break the battery but would be good to protect up to around 50V.
- Over current discharge: We have a 4A fuse on the output. This is fine for now but maybe in the future we can find something that we can recover from. Maybe we can monitor the discharge through the BQ25798 and disable the output if current is too high. Will still always want the fuse though.
- Output short: Similar to over current discharge but will need to be handled slightly differently. Might be that the cell protection chips would trigger from from this and save the fuse from blowing though I expect that we need to have something else to protect/recover from this. Could be that we add an extra external circuit that will detect a voltage spike on the  shunt resistor from the BQ25798 and can this disable the DSG MOSFET.

## Minimum turn on charge

If the battery pack has turned off due to under voltage it will wait until it reaches a set voltage so when a camera is powered back on it has at least a minimum amount of energy to turn on for a desired amount of time.
This was added as sometimes a battery would turn on after charging for a tiny bit and the camera would quickly discharge the battery before it could report events/upload recordings.

## Cell Balancing

The cells get monitored and the maximum and minimum voltage are compared and if the difference is found to be above a set amount then the higher voltage cell will be discharged.
We need to add logic to check if we are doing more cell balancing than we should be if the cells are properly balanced and if so report that to the user in some way. This can happen if cells were not properly placed in the battery pack or if cells are mismatched.

## Safety Testing

These are just the tests for testing that the battery acts in a safe manner.
It does not cover tests that the battery will charge properly using MPPT and such.

### Cell Under Voltage Protection (BQ76920)

Put a cell that is below the minimum voltage and check that it can no longer discharge.
This should also stop the cell balancing from triggering on that cell.
Under voltage protection can also get triggered from the BQ29700 but this is not controlled through software.
You can also set the minim voltage to the maximum value by setting TEST_MINIMUM_CELL_VOLTAGE in bq25798.h

### Cell Over Voltage protection (BQ76920)

Cell Over Voltage Protection should protect the cells from getting charged above a specific voltage.
Need to test charging from the MPPT charger (BQ25798) and getting back powered through the battery output.
Over voltage protection can also get triggered from the BQ29700 but this is not controlled through software.
To help testing this you can use TEST_MAXIMUM_CELL_VOLTAGE in bq25798.h

### Pack over voltage protection

Pack over voltage protection should stop the pack from charging.
Both through the power in and through getting back powered through the battery output.

### Pack under voltage protection

Pack under voltage protection should stop the pack from discharging and put the battery in sleep state to reduce quiescent current.

### Temperature protection

We have two temperature sensors with the cells, one connected to the BQ76920 and one connected to the BQ76920.
These need to be checked every so often (1 second at the moment) and then if below or above a set value we need to stop charging.
Improvements can be made here for reducing the charge/discharge before we get to the upper and lower temperature limits (follow the JEITA recommendations)

### BQ29700 UVP, OVP, OCC, OCD, and SCD

TODO: Figure out a method for testing each [BQ29700](https://www.ti.com/lit/ds/symlink/bq2971.pdf)
This will probably require a bed of nails test fixture to be practical.


## Recovery from battery vault/ protection mode.

- Cell under voltage protection (BQ76920): Charging is still allowed 