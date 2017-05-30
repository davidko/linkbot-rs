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
//! let mut l = Linkbot::new("ZRG6").unwrap();
//! l.move_motors(7, 90.0, 90.0, 90.0);
//! l.move_wait(7);
//! l.move_motors(7, -180.0, -180.0, -180.0);
//! ```

extern crate libc;

use std::sync::{Mutex, Condvar};

/// An object representing a Linkbot.
pub struct Linkbot {
    inner: Box<Inner>
}

#[repr(C)]
#[doc(hidden)]
pub struct Inner {
    c_impl: *mut libc::c_void,
    movewait_mask: (Mutex<u8>, Condvar),
    motor_mask: u8
    //_movewait_channel: Sender<i32, i32>,
}

extern "C" fn linkbot_joint_event_callback(joint_no: i32, event: i32, _: i32, linkbot: *mut Inner)
{
    unsafe {
        let &(ref lock, ref cond) = &((*linkbot).movewait_mask);

        let mut mask = lock.lock().unwrap();
        if (event == 0) || (event == 1) {
            *mask &= !(1<<joint_no);
            cond.notify_all();
        }
        //let mask = (*linkbot).movewait_mask.lock().unwrap();
    }
}

impl Linkbot {
    /// Connect to a Linkbot. 
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
                                    movewait_mask: (Mutex::new(0), Condvar::new()),
                                    motor_mask: 0,
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

    /// Move a Linkbot's motors. The mask specifies which motors to move. For instance, a mask
    /// value of 5 (0b101) indicates that motors 1 and 3 should be moved. A mask of 4 (0b100)
    /// indicates that only motor 3 should be moved.
    ///
    /// The units of movement are in degrees.
    pub fn move_motors(&mut self, mask: u8, j1: f64, j2: f64, j3: f64) {
        let &(ref lock, _) = &(self.inner.movewait_mask);
        {
            let mut _mask = lock.lock().unwrap();
            *_mask |= mask & self.inner.motor_mask;
        }
        unsafe{
            linkbotMove(self.inner.c_impl, mask as u32, j1, j2, j3);
        }
    }

    /// Move a Linkbot's motors to absolute positions.
    ///
    /// See `move_motors` for a description of the mask argument.
    pub fn move_motors_to(&mut self, mask: u8, j1: f64, j2: f64, j3: f64) {
        let &(ref lock, _) = &(self.inner.movewait_mask);
        {
            let mut _mask = lock.lock().unwrap();
            *_mask |= mask & self.inner.motor_mask;
        }
        unsafe{
            linkbotMoveTo(self.inner.c_impl, mask as u32, j1, j2, j3);
        }
    }

    /// Wait until a Linkbot's motors have stopped moving.
    ///
    /// This function blocks execution in the current thread until the Linkbot's motors have
    /// stopped moving. The motors to wait for are specified by the mask argument. For instance, to
    /// wait for all motors, specify a mask of 0x07 (0b111). Or, to only wait for motor 3, specify
    /// a mask of 0x04 (0b100).
    pub fn move_wait(&mut self, mask: u8) {
        let &(ref lock, ref cond) = &(self.inner.movewait_mask);
        let mut _mask = lock.lock().unwrap();
        while (mask & *_mask) != 0 {
            _mask = cond.wait(_mask).unwrap();
        }
    }
}

#[link(name = "linkbot")]
extern {
    fn linkbotFromSerialId(serial_id: *const u8) -> *mut libc::c_void;
    fn linkbotGetFormFactor(linkbot_inner: *mut libc::c_void,
                            form: *mut libc::c_uint);
    fn linkbotMove(linkbot_inner: *mut libc::c_void, 
                   mask: libc::c_uint,
                   j1: libc::c_double,
                   j2: libc::c_double,
                   j3: libc::c_double) -> libc::c_int;
    fn linkbotMoveTo(linkbot_inner: *mut libc::c_void, 
                   mask: libc::c_uint,
                   j1: libc::c_double,
                   j2: libc::c_double,
                   j3: libc::c_double) -> libc::c_int;
    fn linkbotSetJointEventCallback(linkbot_inner: *mut libc::c_void,
                                    cb: extern fn(i32, i32, i32, *mut Inner),
                                    userdata: *mut Inner);
}

#[cfg(test)]
mod tests {
    use super::Linkbot;
    #[test]
    fn it_works() {
        let mut l = Linkbot::new("ZRG6").unwrap();
        l.move_motors(7, 90.0, 90.0, 90.0);
        l.move_wait(7);
        l.move_motors(7, -90.0, -90.0, -90.0);
    }
}
