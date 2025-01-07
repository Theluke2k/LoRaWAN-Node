# What is Pipesense-CLI?
Pipesense-CLI is a Command Line Interface for interacting with the LoraCom modules. The software is written in the Rust programming language. It is a simple client tool that sends a post request to CIBICOMs backend, which subsequently forwards the package to the specified device on its next wake up. The tool works by passing flags and arguments to the program. The software is based on the CLI-interface developed for the LoPy devices used for test in the Pipesense project. A handler for the commands are integrated in the TDR-boards.

# Usage 
## Normal Usage
1. Download the LoPy-cli.exe program from the git repository and save it in a desired folder.
2. Open CMD and navigate to the folder where the program was saved (e.g. "cd C:\Users\user\Downloads").
3. Run the program from CMD (Lopy-cli.exe) and specify the commands (see "Commands"). The general format is "LoPy-cli.exe <Command 1> <Command 2>...". For example, to ping device 6, one would write "LoPy-cli.exe --devices 6 --ping". The "devices" parameter with associated value must always be specified!

## Developers
1. Download the full git repository.
2. Modify the source code as wished.
3. Open CMD and navigate to the repository (e.g. cd C:\Users\user\Downloads\Pipesense-CLI-main).
4. In CMD, run "cargo build --release". Notice that Rust must be installed on the computer.
5. The new program file can be found in the "\Pipesense-CLI-main\target\release" directory.

# Commands 
A list of all the commands implemented in the CLI.

## New Commands for One-String Format
* **--uplink-periodicity \<ms\>:** Sets the uplink periodicity in milliseconds.
* **--set-adr \<1/0\>:** Enables/disables adaptive datarate (ADR) on the device.
* **--set-datarate \<0-5\>:** Sets the datarate for uplinks. 1 is fastest and 5 is slowest. **Note:** only applied if ADR is disabled.
* **--ping \<1/0\>:** Pings the device. Device does not answer anything no matter what. **Note:** Used for debug.
* **--rawlora-duration \<ms\>:** Specifies the duration of a raw LoRa session in milliseconds.
* **--rawlora-periodicity \<ms\>:** Specifies the transmission periodicity in raw LoRa sessions in milliseconds.
* **--rawlora-frequency \<Hertz\>:** Specifies the frequency of transmissions in raw LoRa sessions in Hertz.
* **--rawlora-spreading-factor \<7-12\>:** Specifies the spreading factor of transmissions in raw LoRa sessions.
* **--rawlora-start-in \<ms\>:** Specifies how many milliseconds to wait before starting the next raw LoRa session. 
