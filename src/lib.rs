//! # Robotiq-rs
//! 
//! [![Static Badge](https://img.shields.io/badge/crate-robotiq_rs-red)](https://crates.io/crates/robotiq-rs)
//! [![docs.rs](https://img.shields.io/docsrs/inovo-rs)](https://docs.rs/robotiq-rs/latest/robotiq_rs/)
//! 
//! `robotiq-rs` is a library for interfacing with robotiq gripper
//! ### Compatiable product
//! - [x] Robotiq 2F-85
//! - [x] Robotiq 2F-140 ~(i guess🤷‍♂️)~
//! - [x] HandE
//! - [ ] 3-Finger Gripper
//! - [ ] Vaccum Gripper
//! 
//! ## Example 
//! ```no_run
//! use robotiq_rs::*;
//! 
//! #[tokio::main]
//! async fn main() -> Result<(), RobotiqError> {
//!     // The serial port path
//!     let path = "COM17";
//! 
//!     // create a connection to serial RS485 modbus
//!     let mut gripper = RobotiqGripper::from_path(path)?;
//! 
//!     // Reset and Activation of Gripper
//!     //
//!     // reset gripper, recommended
//!     gripper.reset().await?;
//!     // activate the gripper, it will try to open and close.
//!     gripper.activate().await?.await_activate().await?;
//!     println!("finished activation.");
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     // Basic Gripper Command
//!     //
//!     // set gripper with position, speed and force
//!     gripper.go_to(0x08, 0x00, 0x00).await?;
//!     // the result of the setting command, return whether or not it have clamp an object
//!     let obj_detect_status = gripper.await_go_to().await?;
//!     println!("Object Detect Status : {:?}", obj_detect_status);
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     // Chained command
//!     //
//!     // chained command with builder pattern
//!     let obj_detect_status = gripper.go_to(0xFF, 0xFF, 0xFF).await?.await_go_to().await?;
//!     println!("Object Detect Status : {:?}", obj_detect_status);
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     // Automatic Release Routine
//!     //
//!     // set the gripper into motion
//!     gripper.go_to(0x08, 0x00, 0x00).await?;
//!     std::thread::sleep(std::time::Duration::from_millis(100));
//!     gripper.automatic_release(false).await?;
//!     // you will need to reset and reactivate the gripper after automatic release routine
//!     gripper
//!         .reset()
//!         .await?
//!         .activate()
//!         .await?
//!         .await_activate()
//!         .await?;
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//!     gripper.go_to(0x08, 0x00, 0x00).await?;
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     // Gripper Command
//!     //
//!     // Construct GripperCommand to command gripper
//!     //
//!     // a null command, all zero, will deactivate and reset the gripper
//!     let cmd_null = GripperCommand::new();
//!     // command to activate the gripper
//!     let cmd_act = GripperCommand::new().act(true);
//!     // commands to set the gripper with position requested, speed, and force
//!     let cmd_goto_1 = GripperCommand::new()
//!         .act(true)
//!         .gto(true)
//!         .pos_req(0x08)
//!         .speed(0x00)
//!         .force(0x00);
//!     let cmd_goto_2 = GripperCommand::new()
//!         .act(true)
//!         .gto(true)
//!         .pos_req(0xFF)
//!         .speed(0xFF)
//!         .force(0xFF);
//!     // command to perform automatic release routine
//!     let cmd_atr = GripperCommand::new().act(true).atr(true).ard(true);
//! 
//!     //
//!     //
//!     gripper.write_async(cmd_null).await?;
//!     gripper.write_async(cmd_act).await?;
//!     while gripper.read_async().await?.sta != ActivationStatus::Completed {
//!         std::thread::sleep(std::time::Duration::from_millis(100));
//!     }
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     //
//!     //
//!     gripper.write_async(cmd_goto_1).await?;
//!     while gripper.read_async().await?.obj == ObjDetectStatus::InMotion {
//!         std::thread::sleep(std::time::Duration::from_millis(100));
//!     }
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     //
//!     //
//!     gripper.write_async(cmd_goto_2).await?;
//!     std::thread::sleep(std::time::Duration::from_millis(100));
//!     gripper.write_async(cmd_atr).await?;
//!     while gripper.read_async().await?.fault != GripperFault::AutomaticReleaseCompleted {
//!         std::thread::sleep(std::time::Duration::from_millis(100));
//!     }
//!     std::thread::sleep(std::time::Duration::from_millis(1000));
//! 
//!     gripper
//!         .reset()
//!         .await?
//!         .activate()
//!         .await?
//!         .await_activate()
//!         .await?
//!         .go_to(0x00, 0xFF, 0xFF)
//!         .await?
//!         .await_go_to()
//!         .await?;
//! 
//!     Ok(())
//! }
//! ```


use num::FromPrimitive;
use num_derive::FromPrimitive;
use serde::{Deserialize, Serialize};
use tokio_modbus::prelude::*;
use tokio_serial::SerialPortBuilderExt;

/// Flag for `rACT` and `gACT`
static FLAG_ACT: u8 = 1 << 0;
/// Flag for `rGTO` and `gGTO`
static FLAG_GTO: u8 = 1 << 3;
/// Flag for `rATR`
static FLAG_ATR: u8 = 1 << 4;
/// Flag for `rARD`
static FLAG_ADR: u8 = 1 << 5;

/// The gripper's activation status
#[repr(u8)]
#[derive(Debug, Clone, FromPrimitive, PartialEq, Serialize, Deserialize)]
pub enum ActivationStatus {
    /// Gripper is in reset (or automatic release). See Fault Status if gripper is activated
    InReset,
    // Activation in Progress
    InProgess,
    /// Not used
    NotUsed,
    /// Activation is Completed
    Completed,
}

/// Object Detection status, is a built-in feature that provides information
/// on possible object pick-up. Ignore if `gGTO == 0`.
#[repr(u8)]
#[derive(Debug, Clone, FromPrimitive, PartialEq, Serialize, Deserialize)]
pub enum ObjDetectStatus {
    /// Fingers are in motion towards requested position. No object detected
    InMotion,
    /// Fingers have stopped due to a contact while opening before requested position
    /// Object detected opening.
    DetectedOpen,
    /// Fingers have stopped due to a contact while closing before requested position
    /// Object detected closing.
    DetectedClose,
    /// Fingers are at requested position. No object detected or object has been loss/dropped.
    NoObject,
}

impl ObjDetectStatus {
    pub fn detected_obj(&self) -> bool {
        match self {
            ObjDetectStatus::DetectedClose | ObjDetectStatus::DetectedOpen => true,
            _ => false,
        }
    }
}

/// Fault status return general error messages that are useful for troubleshooting.
/// Fault LED (red) is present on the gripper chassis,
/// LED can be blue, red or both and be solid or blinking.
#[repr(u8)]
#[derive(Debug, Clone, FromPrimitive, PartialEq, Error, Serialize, Deserialize)]
pub enum GripperFault {
    /// No fault (solid blue LED)
    NoFault = 0x00,

    /// Action delayed. the activation (re-activation) must be completed piror to performing the action
    ActionDelay = 0x05,
    /// THe activation bit must be set prior to performing the action
    NotActivated = 0x07,

    /// Maximum operating temperature exceeded (>= 85 degree celsius internally), let cool down (below 80 degree celsius)
    OverHeated = 0x08,
    /// No communication during at least 1 second.
    NoComm = 0x09,

    /// Under minimum operating voltage
    UnderVoltage = 0x0A,
    /// Automatic release in progress
    Releasing = 0x0B,
    /// Internal fault; contact support@robotiq.com
    InternalFault = 0x0C,
    /// Activation fault, verify that no interference or other error occured.
    AcitivationFault = 0x0D,
    /// Overcurrent Triggered.
    OverCurrent = 0x0E,
    /// Automatic release compeleted
    AutomaticReleaseCompleted = 0x0F,
}

impl GripperFault {
    /// For Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit (`rACT`) needed).
    pub fn reset_required(&self) -> bool {
        self.clone() as u8 >= 0x0A
    }
}

impl std::fmt::Display for GripperFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// Robot Input / Status of the gripper
///
/// Read from register starting at `2000`, 6 bytes of data.
/// Detailing the status of gripper, e.g. current position, speed, force ...
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GripperStatus {
    /// Activation status, echo of the `rACT` bit (activation bit).
    pub act: bool,
    /// Action status, echo of the `rGTO` bit (go to bit).
    pub gto: bool,
    /// gripper status, returns the current status and motion of the gripper fingers
    pub sta: ActivationStatus,
    /// Objected detection status, is a built-in feature that provides information on possible object pick-up. Ignore if `gto == 0`
    pub obj: ObjDetectStatus,
    /// Fault status returns general error messages that are useful for troubleshooting.
    pub fault: GripperFault,
    /// Other fault status, the masked higher 4 bits of register `FAULT STATUS`, (mask `0xF0`).
    pub k_flt: u8,
    /// Echo of the requested position for the gripper, value between `0x00` and `0xFF`.
    pub pos_req: u8,
    /// Actual position of the gripper obtained via the encoders, value between `0x00` and `0xFF`.
    pub pos: u8,
    /// The current is read instantaneously form the motor drive, value between `0x00` and `0xFF`, approximate current equivalent is `10 * current` in mA.
    pub current: u8,
}

impl GripperStatus {
    /// parse the gripper status from the registers.
    pub fn parse(bytes: Vec<u8>) -> Self {
        let act = bytes[0] & 1 != 0;
        let gto = bytes[0] & 8 != 0;
        let sta = ActivationStatus::from_u8((bytes[0] >> 4) & 0b11).unwrap();
        let obj = ObjDetectStatus::from_u8((bytes[0] >> 6) & 0b11).unwrap();
        let fault = GripperFault::from_u8(bytes[2]).unwrap();
        let k_flt = bytes[2] & 0xF0;
        let pos_req = bytes[3];
        let pos = bytes[4];
        let current = bytes[5];

        GripperStatus {
            act,
            gto,
            sta,
            obj,
            fault,
            k_flt,
            pos_req,
            pos,
            current,
        }
    }
}

impl From<Vec<u8>> for GripperStatus {
    fn from(value: Vec<u8>) -> Self {
        GripperStatus::parse(value)
    }
}

/// Robot Output/ Functionalities
///
/// Write to registers starting at `1000`, 6 byte of data.
/// specified the command parameter e.g. request position, speed, force ...
///
/// ## Reset
/// To reset the gripper send a command with `act = false`.
///
/// ## Activating the gripper
/// To activate the gripper, send a command with `act = true`.
///
/// `act` need to remain `true` in all following command, otherwise the gripper will reset.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GripperCommand {
    /// First action to be made prior to any other actions, `rACT` bit will activate the gripper.
    /// Clear the `rACT` bit to reset the gripper and clear any fault status
    ///
    /// ## Warning
    /// when setting `act = true`(`rACT == 1`), the gripper will begin movement to complete its activation feature.
    ///
    /// ## Info
    /// Power loss will set `rACT == 1`; the `rACT` bit must be clear `act = false`(`rACT = 0`),
    /// then set again to allow operation of the gripper
    ///
    /// ## Caution
    /// The `rACT` bit must stay on (`act = true`) afterward for any other action to be performed.
    pub act: bool,
    /// The "Go To" action moves the gripper fingers to the requested position using the configuration defined by the other register,
    /// `gto`(`rGTO`) will engage motion while byte 3,4,5 will determine aimed position, force, and speed.
    /// The only motions performed without `gto`(`rGTO`) bit are activation and automation release routines.
    pub gto: bool,
    /// Automatic Release routine action slowly opens the gripper fingers until all motion axes reach their mechanical limits
    /// After all motion are completed, the gripper sends a fault signal and needed to be reinitalized before any other motion is performed.
    /// The `atr`(`rATR`) bit overrides all other commands excluding the activation bit `act`(`rACT`).
    pub atr: bool,
    /// Auto-release direction.
    /// When auto-releasing, `ard`(`rARD`) commands the direction of the movement. The `ard`(`rARD`) bit should be set prior to
    /// or at the same time as the `atr`(`rATR`) bit. as the motion direction is set when the auto-release is initated.
    pub ard: bool,
    /// This register is used to set the target position for the gripper's fingers.
    /// The position `0x00` and `0xFF` correspond respectively to the fully opened and fully closed mechanical stops.
    /// For detail finger trajectory, please refer to the Specifications section of repective manual.
    ///
    /// - `0x00` - Open position
    /// - `0xFF` - Close position
    ///
    /// ## Info
    /// The activation will allow the gripper to adjust to any fingertips.
    /// No matter what is the size and/or shape of the fingertips,
    /// `0` will always be fully opened and `255` fully closed,
    /// with a quasi-linear relationship between the two values.
    pub pos_req: u8,
    /// This register is used to set the gripper closing and opening speed in real time,
    /// however, setting a speed will not initiate a motion.
    ///
    /// - `0x00` - Minimum speed
    /// - `0xFF` - Maximum speed
    pub speed: u8,
    /// The force setting defines the final gripper force for the gripper.
    /// The force will fix the maximum current sent to the motor.
    /// If the current limit is exceeded, the fingers stop and trigger an object detection notification.
    /// Please refer to the "Picking Features" section of respective manual.
    ///
    /// - `0x00`- Minimum force
    /// - `0xFF`- Maximum force
    pub force: u8,
}

impl GripperCommand {
    /// Create a new default gripper command.
    pub fn new() -> Self {
        Self::default()
    }

    /// First action to be made prior to any other actions, `rACT` bit will activate the gripper.
    /// Clear the `rACT` bit to reset the gripper and clear any fault status
    ///
    /// ## Warning
    /// when setting `act = true`(`rACT == 1`), the gripper will begin movement to complete its activation feature.
    ///
    /// ## Info
    /// Power loss will set `rACT == 1`; the `rACT` bit must be clear `act = false`(`rACT = 0`),
    /// then set again to allow operation of the gripper
    ///
    /// ## Caution
    /// The `rACT` bit must stay on (`act = true`) afterward for any other action to be performed.
    pub fn act(mut self, b: bool) -> Self {
        self.act = b;
        self
    }
    /// The "Go To" action moves the gripper fingers to the requested position using the configuration defined by the other register,
    /// `gto`(`rGTO`) will engage motion while byte 3,4,5 will determine aimed position, force, and speed.
    /// The only motions performed without `gto`(`rGTO`) bit are activation and automation release routines.
    pub fn gto(mut self, b: bool) -> Self {
        self.gto = b;
        self
    }
    /// Automatic Release routine action slowly opens the gripper fingers until all motion axes reach their mechanical limits
    /// After all motion are completed, the gripper sends a fault signal and needed to be reinitalized before any other motion is performed.
    /// The `atr`(`rATR`) bit overrides all other commands excluding the activation bit `act`(`rACT`).
    pub fn atr(mut self, b: bool) -> Self {
        self.atr = b;
        self
    }
    /// Auto-release direction.
    ///
    /// When auto-releasing, `ard`(`rARD`) commands the direction of the movement. The `ard`(`rARD`) bit should be set prior to
    /// or at the same time as the `atr`(`rATR`) bit. as the motion direction is set when the auto-release is initated.
    pub fn ard(mut self, b: bool) -> Self {
        self.ard = b;
        self
    }
    /// This register is used to set the target position for the gripper's fingers.
    /// The position `0x00` and `0xFF` correspond respectively to the fully opened and fully closed mechanical stops.
    /// For detail finger trajectory, please refer to the Specifications section of repective manual.
    ///
    /// - `0x00` - Open position
    /// - `0xFF` - Close position
    ///
    /// ## Info
    /// The activation will allow the gripper to adjust to any fingertips.
    /// No matter what is the size and/or shape of the fingertips,
    /// `0` will always be fully opened and `255` fully closed,
    /// with a quasi-linear relationship between the two values.
    pub fn pos_req(mut self, b: u8) -> Self {
        self.pos_req = b;
        self
    }
    /// This register is used to set the gripper closing and opening speed in real time,
    /// however, setting a speed will not initiate a motion.
    ///
    /// - `0x00` - Minimum speed
    /// - `0xFF` - Maximum speed
    pub fn speed(mut self, b: u8) -> Self {
        self.speed = b;
        self
    }
    /// The force setting defines the final gripper force for the gripper.
    /// The force will fix the maximum current sent to the motor.
    /// If the current limit is exceeded, the fingers stop and trigger an object detection notification.
    /// Please refer to the "Picking Features" section of respective manual.
    ///
    /// - `0x00`- Minimum force
    /// - `0xFF`- Maximum force
    pub fn force(mut self, b: u8) -> Self {
        self.force = b;
        self
    }
    /// make array for writing to register.
    pub fn to_array(&self) -> [u16; 3] {
        let mut req = 0;

        if self.act {
            req |= FLAG_ACT;
        }
        if self.gto {
            req |= FLAG_GTO;
        }
        if self.atr {
            req |= FLAG_ATR;
        }
        if self.ard {
            req |= FLAG_ADR;
        }

        [
            u16::from_be_bytes([req, 0]),
            u16::from_be_bytes([0, self.pos_req]),
            u16::from_be_bytes([self.speed, self.force]),
        ]
    }
}

/// Data Structure for interfacing with robotiq gripper with modbus rtu protocol.
pub struct RobotiqGripper {
    ctx: client::Context,
}

impl RobotiqGripper {
    /// The Default Modbus slave ID of robotiq gripper
    pub const DEFAULT_SLAVE_ID: u8 = 9;

    /// Constructer from a modbus context.
    pub fn new(ctx: client::Context) -> Self {
        Self { ctx }
    }

    /// Constructer from USB port path, and slave id.
    pub fn from_path_slave_id(
        path: impl Into<String>,
        slave_id: u8,
    ) -> Result<Self, std::io::Error> {
        let port = tokio_serial::new(path.into(), 115_200)
            .data_bits(tokio_serial::DataBits::Eight)
            .stop_bits(tokio_serial::StopBits::One)
            .parity(tokio_serial::Parity::None)
            .timeout(std::time::Duration::from_millis(500))
            .open_native_async()?;

        let ctx = rtu::attach_slave(port, Slave(slave_id));

        Ok(Self::new(ctx))
    }

    /// Constructer form USB port path.
    pub fn from_path(path: impl Into<String>) -> Result<Self, std::io::Error> {
        Self::from_path_slave_id(path, Self::DEFAULT_SLAVE_ID)
    }

    /// Async function for writing a `GripperCommand` to the gripper
    pub async fn write_async(&mut self, cmd: GripperCommand) -> Result<(), RobotiqError> {
        Ok(self
            .ctx
            .write_multiple_registers(1000, &cmd.to_array())
            .await??)
    }
    /// Async function for reading `GripperCommand` from the gripper
    pub async fn read_async(&mut self) -> Result<GripperStatus, RobotiqError> {
        Ok(self
            .ctx
            .read_holding_registers(2000, 3)
            .await??
            .into_iter()
            .map(|u| u.to_be_bytes())
            .flatten()
            .collect::<Vec<_>>()
            .into())
    }

    // /// Sync function for reading `GripperCommand` from the gripper
    // pub fn read(&mut self) -> Result<GripperStatus, RobotiqError> {
    //     let res = tokio::task::block_in_place(move || {
    //         tokio::runtime::Handle::current().block_on(async { self.read_async().await })
    //     })?;
    //     Ok(res.into())
    // }
    // /// Sync function for writing a `GripperCommand` to the gripper
    // pub fn write(&mut self, cmd: GripperCommand) -> Result<(), RobotiqError> {
    //     tokio::task::block_in_place(move || {
    //         tokio::runtime::Handle::current().block_on(async { self.write_async(cmd).await })
    //     })
    // }

    /// Turn off the activation bit `rACT`, to reset fault.
    pub async fn reset(&mut self) -> Result<&mut Self, RobotiqError> {
        self.write_async(GripperCommand::default()).await?;
        Ok(self)
    }
    /// Turn on the activation bit `rACT` to start initalization of the gripper
    pub async fn activate(&mut self) -> Result<&mut Self, RobotiqError> {
        self.write_async(GripperCommand::new().act(true)).await?;
        Ok(self)
    }
    /// Await for the initalization process to finish.
    pub async fn await_activate(&mut self) -> Result<&mut Self, RobotiqError> {
        loop {
            let status = self.read_async().await?;
            if status.sta == ActivationStatus::Completed {
                break;
            }
            if status.fault != GripperFault::NoFault {
                return Err(status.fault.into());
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        Ok(self)
    }
    /// Command the gripper to go to a certain set point with specified position, speed and force
    ///
    /// ## Parameter
    /// ### `pos_req: u8`
    /// This register is used to set the target position for the gripper's fingers.
    /// The position `0x00` and `0xFF` correspond respectively to the fully opened and fully closed mechanical stops.
    /// For detail finger trajectory, please refer to the Specifications section of repective manual.
    ///
    /// - `0x00` - Open position
    /// - `0xFF` - Close position
    ///
    /// ## Info
    /// The activation will allow the gripper to adjust to any fingertips.
    /// No matter what is the size and/or shape of the fingertips,
    /// `0` will always be fully opened and `255` fully closed,
    /// with a quasi-linear relationship between the two values.
    /// ### `speed: u8`
    /// This register is used to set the gripper closing and opening speed in real time,
    /// however, setting a speed will not initiate a motion.
    ///
    /// - `0x00` - Minimum speed
    /// - `0xFF` - Maximum speed
    /// ### `force: u8`
    /// The force setting defines the final gripper force for the gripper.
    /// The force will fix the maximum current sent to the motor.
    /// If the current limit is exceeded, the fingers stop and trigger an object detection notification.
    /// Please refer to the "Picking Features" section of respective manual.
    ///
    /// - `0x00`- Minimum force
    /// - `0xFF`- Maximum force
    pub async fn go_to(
        &mut self,
        pos_req: u8,
        speed: u8,
        force: u8,
    ) -> Result<&mut Self, RobotiqError> {
        let cmd = GripperCommand::new()
            .act(true)
            .gto(true)
            .pos_req(pos_req)
            .speed(speed)
            .force(force);
        self.write_async(cmd).await?;
        Ok(self)
    }
    /// Await for the "Go To" command to finish.
    ///
    /// ## Return `ObjDetectStatus`
    /// this function will return, when the gripper detected object at the end point, or reached target position.
    pub async fn await_go_to(&mut self) -> Result<ObjDetectStatus, RobotiqError> {
        loop {
            let status = self.read_async().await?;
            if status.obj != ObjDetectStatus::InMotion {
                self.activate().await?;
                return Ok(status.obj);
            }
            if status.fault != GripperFault::NoFault {
                return Err(status.fault.into());
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
    /// ### Automatic release feature.
    ///
    /// Automatic Release routine action slowly opens the gripper fingers until all motion axes reach their mechanical limits
    /// After all motion are completed, the gripper sends a fault signal and needed to be reinitalized before any other motion is performed.
    /// The `atr`(`rATR`) bit overrides all other commands excluding the activation bit `act`(`rACT`).
    ///
    /// #### Auto-release direction.
    ///
    /// When auto-releasing, `ard`(`rARD`) commands the direction of the movement. The `ard`(`rARD`) bit should be set prior to
    /// or at the same time as the `atr`(`rATR`) bit. as the motion direction is set when the auto-release is initated.
    pub async fn automatic_release(&mut self, open: bool) -> Result<&mut Self, RobotiqError> {
        let cmd = GripperCommand::new().act(true).atr(true).ard(open);
        self.write_async(cmd).await?;
        Ok(self)
    }

    /// Await for the automatic release routine to finish.
    pub async fn await_automatic_release(&mut self) -> Result<&mut Self, RobotiqError> {
        loop {
            let status = self.read_async().await?;
            match status.fault {
                GripperFault::Releasing => {}
                GripperFault::AutomaticReleaseCompleted => return Ok(self),
                _ => return Err(status.fault.into()),
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}

impl Drop for RobotiqGripper {
    fn drop(&mut self) {
        tokio::task::block_in_place(move || {
            tokio::runtime::Handle::current().block_on(async move {
                let _ = self.ctx.disconnect().await;
            });
        });
    }
}

use thiserror::Error;

#[derive(Debug, Error)]
pub enum RobotiqError {
    #[error("std io error, serial comm error")]
    IOError(#[from] std::io::Error),
    #[error("Modbus protocol or transport errros.")]
    ModbusError(#[from] tokio_modbus::Error),
    #[error("A server (slave) exception.")]
    ModbusException(#[from] tokio_modbus::Exception),
    #[error("gripper fault")]
    GripperFault(#[from] GripperFault),
}
