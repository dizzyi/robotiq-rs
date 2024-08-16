use robotiq_rs::*;

#[tokio::main]
async fn main() -> Result<(), RobotiqError> {
    // The serial port path
    let path = "COM17";

    // create a connection to serial RS485 modbus
    let mut gripper = RobotiqGripper::from_path(path)?;

    // Reset and Activation of Gripper
    //
    // reset gripper, recommended
    gripper.reset().await?;
    // activate the gripper, it will try to open and close.
    gripper.activate().await?.await_activate().await?;
    println!("finished activation.");
    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Basic Gripper Command
    //
    // set gripper with position, speed and force
    gripper.go_to(0x08, 0x00, 0x00).await?;
    // the result of the setting command, return whether or not it have clamp an object
    let obj_detect_status = gripper.await_go_to().await?;
    println!("Object Detect Status : {:?}", obj_detect_status);
    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Chained command
    //
    // chained command with builder pattern
    let obj_detect_status = gripper.go_to(0xFF, 0xFF, 0xFF).await?.await_go_to().await?;
    println!("Object Detect Status : {:?}", obj_detect_status);
    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Automatic Release Routine
    //
    // set the gripper into motion
    gripper.go_to(0x08, 0x00, 0x00).await?;
    std::thread::sleep(std::time::Duration::from_millis(100));
    gripper.automatic_release(false).await?;
    // you will need to reset and reactivate the gripper after automatic release routine
    gripper
        .reset()
        .await?
        .activate()
        .await?
        .await_activate()
        .await?;
    std::thread::sleep(std::time::Duration::from_millis(1000));
    gripper.go_to(0x08, 0x00, 0x00).await?;
    std::thread::sleep(std::time::Duration::from_millis(1000));

    // Gripper Command
    //
    // Construct GripperCommand to command gripper
    //
    // a null command, all zero, will deactivate and reset the gripper
    let cmd_null = GripperCommand::new();
    // command to activate the gripper
    let cmd_act = GripperCommand::new().act(true);
    // commands to set the gripper with position requested, speed, and force
    let cmd_goto_1 = GripperCommand::new()
        .act(true)
        .gto(true)
        .pos_req(0x08)
        .speed(0x00)
        .force(0x00);
    let cmd_goto_2 = GripperCommand::new()
        .act(true)
        .gto(true)
        .pos_req(0xFF)
        .speed(0xFF)
        .force(0xFF);
    // command to perform automatic release routine
    let cmd_atr = GripperCommand::new().act(true).atr(true).ard(true);

    //
    //
    gripper.write_async(cmd_null).await?;
    gripper.write_async(cmd_act).await?;
    while gripper.read_async().await?.sta != ActivationStatus::Completed {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    std::thread::sleep(std::time::Duration::from_millis(1000));

    //
    //
    gripper.write_async(cmd_goto_1).await?;
    while gripper.read_async().await?.obj == ObjDetectStatus::InMotion {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    std::thread::sleep(std::time::Duration::from_millis(1000));

    //
    //
    gripper.write_async(cmd_goto_2).await?;
    std::thread::sleep(std::time::Duration::from_millis(100));
    gripper.write_async(cmd_atr).await?;
    while gripper.read_async().await?.fault != GripperFault::AutomaticReleaseCompleted {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    std::thread::sleep(std::time::Duration::from_millis(1000));

    gripper
        .reset()
        .await?
        .activate()
        .await?
        .await_activate()
        .await?
        .go_to(0x00, 0xFF, 0xFF)
        .await?
        .await_go_to()
        .await?;

    Ok(())
}
