//! Control Barobo Linkbots with Rust
//!
//! This library is a thin wrapper around Barobo's C++ Linkbot library. It provides a way to
//! control Linkbot peripherals and get data from a Linkbot with Rust.
//!
//! You can find more information about Linkbots at https://barobo.com .
//!
//! ## Installation
//!
//! This crate requires the "liblinkbot" libraries to be installed. On Ubuntu, you can install
//! liblinkbot with the following commands:
//!
//! ```text
//! wget http://repo.barobo.com/barobo.public.key -O - | sudo apt-key add -
//! sudo add-apt-repository "deb http://repo.barobo.com/ xenial main"
//! sudo apt-get update
//! sudo apt-get install liblinkbot
//! ```
//!
//! ## Examples
//!
//! Following is a simple example which connects to robot "ZRG6" and moves all of its motors in the
//! positive direction by 90 degrees. After the first motion is completed, it moves all of its
//! motors in the negative direction by 180 degrees.
//!
//! ```
//! extern crate linkbot;
//!
//! use linkbot::Linkbot;
//!
//! fn main() {
//!     let mut l = Linkbot::new("ZRG6").unwrap();
//!     l.move_motors(7, 90.0, 90.0, 90.0);
//!     l.move_wait(7);
//!     l.move_motors(7, -180.0, -180.0, -180.0);
//! }
//! ```

extern crate libc;

use std::sync::{Mutex, Condvar};

/// An object representing a Linkbot.
pub struct Linkbot {
    inner: Box<Inner>
}

/// Linkbot button
///
/// Used in Linkbot button callback functions. See `set_button_handler`.
pub enum Button {
    Power,
    A,
    B
}

/// Linkbot button state
///
/// Used in Linkbot button callback functions. See `set_button_handler`.
pub enum ButtonState {
    Up,
    Down
}

pub type ButtonCallback = FnMut(Button, ButtonState, u32);

#[repr(C)]
struct MoveWaitMask {
    lock: Mutex<u8>,
    cond: Condvar
}

#[repr(C)]
#[doc(hidden)]
pub struct Inner {
    c_impl: *mut libc::c_void,
    movewait_mask: MoveWaitMask,
    motor_mask: u8,
    button_callback: Option<Box<ButtonCallback>>
}

extern "C" fn linkbot_joint_event_callback(joint_no: i32, event: i32, _: i32, linkbot: *mut Inner)
{
    unsafe {
        let &ref lock = &((*linkbot).movewait_mask.lock);
        let &ref cond = &((*linkbot).movewait_mask.cond);

        let mut mask = lock.lock().unwrap();
        if (event == 0) || (event == 1) {
            *mask &= !(1<<joint_no);
            cond.notify_all();
        }
    }
}

extern "C" fn linkbot_button_event_callback(button: i32, state: i32, timestamp: i32, linkbot: *mut Inner)
{
    unsafe {
    match (*linkbot).button_callback {
        None => { return; }
        Some(ref mut callback) => {
            let _button = match button {
                0 => Button::Power,
                1 => Button::A,
                _ => Button::B
            };
            let _state = match state {
                0 => ButtonState::Up,
                _ => ButtonState::Down
            };
            callback(_button, _state, timestamp as u32);
        }
    } 
    } // unsafe
}

impl Linkbot {
    /// Try to connect to a Linkbot. 
    ///
    /// If there is an error, `None` is returned and an error message is printed to stderr.
    pub fn new(serial_id: &str) -> Option<Linkbot> {
        let _serial_id = serial_id.to_string();
        let _serial_id = _serial_id + "\0";
        unsafe {
            let inner = linkbotFromSerialId((&_serial_id).as_ptr());
            if inner.is_null() {
                None
            } else {
                // Enable joint events
                let linkbot = Inner{c_impl: inner,
                                    movewait_mask: MoveWaitMask{lock: Mutex::new(0), cond: Condvar::new()},
                                    motor_mask: 0,
                                    button_callback: None
                };
                let mut linkbot_boxed = Box::new(linkbot);
                linkbotSetJointEventCallback(linkbot_boxed.c_impl, 
                                             linkbot_joint_event_callback,
                                             &mut *linkbot_boxed);
                let mut form:u32 = 0;
                linkbotGetFormFactor(linkbot_boxed.c_impl, &mut form);
                match form {
                    0 => linkbot_boxed.motor_mask = 0x05,
                    1 => linkbot_boxed.motor_mask = 0x03,
                    3 => linkbot_boxed.motor_mask = 0x07,
                    _ => linkbot_boxed.motor_mask = 0,
                }

                Some(Linkbot{ inner: linkbot_boxed })
            }
        }
    }

    /// Get a Linkbot's current joint angles.
    ///
    /// Returns a tuple containing the three joint angles in degrees and a timestamp representing
    /// the number of milliseconds the robot has been powered on.
    pub fn get_joint_angles(&mut self) -> Result<(f64, f64, f64, u32), i32> {
        let mut j1:f64 = 0.0;
        let mut j2:f64 = 0.0;
        let mut j3:f64 = 0.0;
        let mut timestamp:i32 = 0;

        let rc = unsafe {
            linkbotGetJointAngles( self.inner.c_impl, 
                                   &mut timestamp,
                                   &mut j1,
                                   &mut j2,
                                   &mut j3 )
        };
        if rc == 0 {
            Ok((j1, j2, j3, timestamp as u32))
        } else {
            Err(rc)
        }
    }

    /// Move a Linkbot's motors. The mask specifies which motors to move. For instance, a mask
    /// value of 5 (0b101) indicates that motors 1 and 3 should be moved. A mask of 4 (0b100)
    /// indicates that only motor 3 should be moved.
    ///
    /// The units of movement are in degrees.
    pub fn move_motors(&mut self, mask: u8, j1: f64, j2: f64, j3: f64) -> Result<(), i32> {
        let &ref lock = &(self.inner.movewait_mask.lock);
        {
            let mut _mask = lock.lock().unwrap();
            *_mask |= mask & self.inner.motor_mask;
        }
        let rc = unsafe{
            linkbotMove(self.inner.c_impl, mask as u32, j1, j2, j3)
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(rc)
        }
    }

    /// Move a Linkbot's motors to absolute positions.
    ///
    /// See `move_motors` for a description of the mask argument.
    pub fn move_motors_to(&mut self, mask: u8, j1: f64, j2: f64, j3: f64) -> Result<(), i32> {
        let &ref lock = &(self.inner.movewait_mask.lock);
        {
            let mut _mask = lock.lock().unwrap();
            *_mask |= mask & self.inner.motor_mask;
        }
        let rc = unsafe{
            linkbotMoveTo(self.inner.c_impl, mask as u32, j1, j2, j3)
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(rc)
        }
    }

    /// Wait until a Linkbot's motors have stopped moving.
    ///
    /// This function blocks execution in the current thread until the Linkbot's motors have
    /// stopped moving. The motors to wait for are specified by the mask argument. For instance, to
    /// wait for all motors, specify a mask of 0x07 (0b111). Or, to only wait for motor 3, specify
    /// a mask of 0x04 (0b100).
    pub fn move_wait(&mut self, mask: u8) -> Result<(), i32> {
        let &ref lock = &(self.inner.movewait_mask.lock);
        let &ref cond = &(self.inner.movewait_mask.cond);
        let mut _mask = lock.lock().unwrap();
        while (mask & *_mask) != 0 {
            _mask = cond.wait(_mask).unwrap();
        }
        Ok(())
    }

    /// Set a callback for when a Linkbot button is pressed
    ///
    /// The callback will be called whenever a button is depressed or released.
    /// The third argument of the callback function is a timestamp indicating the number of
    /// milleseconds the robot has been powered on.
    ///
    /// # Examples
    /// ```
    /// extern crate linkbot;
    /// use linkbot::Linkbot;
    ///
    /// fn main() {
    ///     let mut l = Linkbot::new("ZRG6").unwrap();
    ///     l.set_button_handler( |button, state, timestamp| {
    ///         // Do something. This closure will be called when any button is depressed or
    ///         // released.
    ///     });
    /// }
    pub fn set_button_handler<F>(&mut self, callback: F) -> Result<(), i32>
        where F: FnMut(Button, ButtonState, u32) + 'static
    {
        self.inner.button_callback = Some(Box::new(callback));
        let rc = unsafe {
            linkbotSetButtonEventCallback(
                self.inner.c_impl,
                Some(linkbot_button_event_callback),
                &mut *self.inner)
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(rc)
        }
    }

    /// Remove the Linkbot button handler
    ///
    /// If there was a button handler previously set with `set_button_handler()`, this function
    /// removes it and restores the default Linkbot button functionality.
    pub fn unset_button_handler(&mut self) -> Result<(), i32>
    {
        let rc = unsafe {
            linkbotSetButtonEventCallback(
                self.inner.c_impl,
                None,
                &mut *self.inner)
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(rc)
        }
    }

    /// Set the buzzer frequency
    ///
    /// Set the buzzer frequency to a value specified in Hertz (Hz). Set the buzzer frequency to 0
    /// to stop the buzzer.
    pub fn set_buzzer_frequency(&mut self, frequency: f32) -> Result<(), i32> {
        let rc = unsafe {
            linkbotSetBuzzerFrequency(
                self.inner.c_impl,
                frequency)
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(rc)
        }
    }
}

#[link(name = "linkbot")]
extern {
    fn linkbotFromSerialId(serial_id: *const u8) -> *mut libc::c_void;
    fn linkbotGetFormFactor(linkbot_inner: *mut libc::c_void,
                            form: *mut libc::c_uint) -> i32;
    fn linkbotGetJointAngles(linkbot_inner: *mut libc::c_void,
                             timestamp: *mut libc::c_int,
                             j1: *mut libc::c_double,
                             j2: *mut libc::c_double,
                             j3: *mut libc::c_double) -> i32;
    fn linkbotMove(linkbot_inner: *mut libc::c_void, 
                   mask: libc::c_uint,
                   j1: libc::c_double,
                   j2: libc::c_double,
                   j3: libc::c_double) -> i32;
    fn linkbotMoveTo(linkbot_inner: *mut libc::c_void, 
                   mask: libc::c_uint,
                   j1: libc::c_double,
                   j2: libc::c_double,
                   j3: libc::c_double) -> i32;
    fn linkbotSetJointEventCallback(linkbot_inner: *mut libc::c_void,
                                    cb: extern fn(i32, i32, i32, *mut Inner),
                                    userdata: *mut Inner) -> i32;
    fn linkbotSetButtonEventCallback(linkbot_inner: *mut libc::c_void,
                                     cb: Option<extern fn(i32, i32, i32, *mut Inner)>,
                                     userdata: *mut Inner) -> i32;
    fn linkbotSetBuzzerFrequency(linkbot_inner: *mut libc::c_void,
                                 frequency: libc::c_float) -> i32;
}

#[cfg(test)]
mod tests {
    use super::Linkbot;
    use std;
    #[test]
    fn it_works() {
        let mut l = Linkbot::new("ZRG6").unwrap();
        l.set_button_handler( |button, state, timestamp| {
            println!("Button press!");
        });
        let (j1, j2, j3, _) = l.get_joint_angles().unwrap();
        println!("Joint angles: {}, {}, {}", j1, j2, j3);
        l.move_motors(7, 90.0, 90.0, 90.0);
        l.move_wait(7);
        l.move_motors(7, -90.0, -90.0, -90.0);
        l.move_wait(7);
        l.unset_button_handler();
        l.set_buzzer_frequency(440.0);
        std::thread::sleep( std::time::Duration::from_secs( 1 ) );
        l.set_buzzer_frequency(0.0);
    }
}
