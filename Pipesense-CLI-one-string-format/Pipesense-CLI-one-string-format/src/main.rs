use std::io::{self, Read, Write};
use serde_json::to_string;
use structopt::StructOpt;
use chrono::{DateTime, TimeZone, Utc, FixedOffset};
mod config;
mod api_calls;


fn main() {
    let args = config::Config::from_args();

    let mut devarr: Vec<String> = vec![];

    for dev in args.devices{
        match dev.as_str() {
            "1" => devarr.push("70B3D54996A7C39E".to_string()),
            "2" => devarr.push("BE7A000000000212".to_string()),
            "3" => devarr.push("BE7A000000000210".to_string()),
            "4" => devarr.push("BE7A000000000213".to_string()),
            "5" => devarr.push("BE7A000000000214".to_string()),
            "6" => devarr.push("1000000000000004".to_string()),
            "7" => devarr.push("1000000000000005".to_string()),
            "8" => devarr.push("1000000000000006".to_string()),
            _ => panic!("Invalid input, Aborting")
        }
    };

    println!("DevArr {:?}", devarr);

    // Create vector of string to hold command string/strings.
    let mut command_strings: Vec<String> = vec![String::new()];
    // Keep track of size of current command string
    let mut current_string_size: usize = 0;
    // Maximum byte size of command string
    let max_string_size: usize = 50;

    // Define closure that checks if a command, appended to the command_string, exceeds 50 bytes
    let mut append_command_checked = |new_command: String| {
        /*
        Compute size of new command, also accounting for the ; which will be added, if the command
        string is not empty.
         */
        let new_command_size = new_command.len() + if !command_strings.last().unwrap().is_empty() { 1 } else { 0 };

        // Check if the command size plus command string size exceeds 50 bytes
        if current_string_size + new_command_size > max_string_size {
            // Create new command string if old string will exceed 50 bytes
            command_strings.push(String::new());

            //Reset string size counter
            current_string_size = 0;
        }

        // Add the command to the last command string in our vector of command strings
        append_command(command_strings.last_mut().unwrap(), new_command);
        
        // Update current string size
        current_string_size += new_command_size;
    };
    

    /* ARGUMENT PROCESSING */

    // Periodicity of LoRaWAN uplink
    if args.uplink_periodicity > 0 {
        if(args.uplink_periodicity < 10000) {
            println!("ERROR: uplink_periodicity has to be larger than 10000 ms!");
            return;
        }
        let uplink_periodicity_command = format!("UpPe={}", args.uplink_periodicity);
        append_command_checked(uplink_periodicity_command);
    }

    // Ping device
    if args.ping{
        let ping_command = format!("Pi={}", args.ping);
        append_command_checked(ping_command);
    }   

    // Enable/disable Adaptive Data Rate in LoRaWAN
    if args.set_adr <= 1 {
        let set_adr_command = format!("SeAd={}", args.set_adr);
        append_command_checked(set_adr_command);
    }

    // Choose datarate in LoRaWAN
    if args.set_datarate <= 5 {
        let set_datarate_command = format!("SeDa={}", args.set_datarate);
        append_command_checked(set_datarate_command);
    }



    /* New commands for Raw LoRa */
    // Duration of rawLoRa session
    if args.rawlora_duration >= 0 {
        if(args.rawlora_duration < 5000) {
            println!("ERROR: rawlora_duration has to be larger than 5000 ms!");
            return;
        }
        // TODO:Validity check here?
        let rawlora_duration_command = format!("RaLoDu={}", args.rawlora_duration);
        append_command_checked(rawlora_duration_command);
    }

    // Transmission periodicity in rawLoRa session
    if args.rawlora_periodicity >= 0 {
        if(args.rawlora_periodicity < 5000) {
            println!("ERROR: rawlora-periodicity has to be larger than 5000 ms!");
            return;
        }
        // TODO:Validity check here?
        let rawlora_periodicity_command = format!("RaLoPe={}", args.rawlora_periodicity);
        append_command_checked(rawlora_periodicity_command);
    }

    // Frequency of rawLoRa transmission
    if args.rawlora_frequency >= 0 {
        // TODO:Validity check here?
        let rawlora_frequency_command = format!("RaLoFr={}", args.rawlora_frequency);
        append_command_checked(rawlora_frequency_command);
    }

    // Spreading factor in rawLoRa session
    if args.rawlora_spreading_factor >= 0 {
        // TODO:Validity check here?
        let rawlora_spreading_factor_command = format!("RaLoSF={}", args.rawlora_spreading_factor);
        append_command_checked(rawlora_spreading_factor_command);
    }


    // ALWYAS PLACE THIS AS THE LAST PARAMETER. In that way the config commands are received first.
    // Amount of time before rawLoRa session should start.
    if args.rawlora_start_in >= 0 {
        if(args.rawlora_start_in < 5000) {
            println!("ERROR: rawlora_start_in has to be larger than 5000 ms!");
            return;
        }
        // TODO:Validity check here?
        //api_calls::rawlora_start_in(args.rawlora_start_in);
        let rawlora_start_in_command = format!("RaLoStIn={}", args.rawlora_start_in);
        append_command_checked(rawlora_start_in_command);
    }

    // Check the number of command strings
    if command_strings.len() > 1 {
        println!(
            "The command has been split into {} parts. Are you sure you want to proceed? (y/n)",
            command_strings.len()
        );

        let mut user_input = String::new();
        io::stdin().read_line(&mut user_input).expect("Failed to read input");
        let user_input = user_input.trim().to_lowercase();

        if user_input != "y" {
            println!("Aborting as per user request.");
            return;
        }
    }

    // Iterate through command strings and call API function to send them
    for (i, cmd_string) in command_strings.iter().enumerate() {
        //println!("Command String {}: {}", i+1, cmd_string); // DEBUG print command string(s)

        // Schedule command string as downlink
        api_calls::send_command_string(cmd_string);

    }
}

fn append_command(command_string: &mut String, new_command: String) {
    // If command string is not empty, add a seperator
    if !command_string.is_empty() {
        command_string.push(';'); 
    }

    // Append the new command.
    command_string.push_str(&new_command);
}