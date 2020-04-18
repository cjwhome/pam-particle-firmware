/**
 * PAMSerial - Created 04/18/2020
 * Author: David Kopala
 * 
 * While the Serial class from Arduino works well for smaller applications, it can
 * be hard to use for larger projects that do more than reading and writing the
 * same thing every time. If we want to be able to respond to user input, or filter
 * which pieces of the program can write to the port, we need to keep track of more
 * infromation, which can make the main file cluttered.
 * 
 * For the PAM, the Serial console provides the following features:
 *      1) View data that is being collected in real time
 *      2) Configure device and calibrations settings
 * Most of the time, the PAM will just be printing the values of each specie on the
 * port as a comma delimitted string. When the user presses a button, they open the
 * Serial Menu, which prevents the species' values from being printed and waits
 * for the user to enter a command. A command is typically a single character, and is
 * used to determine some action to take, like getting a new sensor calibration value
 * from the user. After the user exits the Serial Menu, the PAM starts printing the
 * species values again.
 * 
 * The current implementation checks for a new character from the user every loop. If
 * there is a character and it's an 'm', it enters the Serial Menu, which loops until
 * it receives an 'x' from the user. When it receives a character other than an 'x',
 * a function that's identified from a lot of if/else statements is executed. Most of
 * these functions look pretty similar and are used to set the calibration values;
 * they read the current setting to the user and prompt them for a new setting. Once
 * a valid new setting is received, we return control to the Serial Menu loop.
 * 
 * Currently, all of the code that requires the user to enter value blocks until it
 * receives something from the user. While this does work, it doesn't allow the system
 * to service anything else while waiting for input from the user. For example, any
 * time that the Serial Menu is open, no data is advertised anywhere (BLE, LTE,SD Card, etc).
 * And while the device should still respond to interrupts, any operations in response
 * to polling won't happen either, like sleeping if the battery drops below 20%.
 * 
 * It's also worth noting that for a single level menu like what we have now, only 62
 * different options can be selected if you limit the input to a single alphanumeric
 * character, which can be filled rather quickly.
 * 
 * Operating systems provide similar features to what we are looking for: a way to acccept
 * user input that doesn't block other tasks. They accomplish this by using a standardized
 * I/O redirection pattern. Each program reads input from STDIN (fd 0) and outputs on STDOUT
 * (fd 1). When a process "forks" and creates a child process, it shares the same STDIN and
 * STDOUT as its parent process (unless the user has manually setup different pipes in the 
 * shell). After forking, the parent process will generally block until the child process
 * terminates, which essentially provides mutually exclusive access to STDIN/STDOUT to the
 * child process.
 * 
 * To address the issues described above while still providing support for current features,
 * this class will provide a wrapper around the native Serial class that introduces some
 * new capabilities.
 * 
 * Unfortunately, since Arduino doesn't support multitasking, we have to register callback
 * functions instead of being able to wait for a response in a block. This will increase
 * complexity of the modules receiving the user input, but cannot be avoided if we don't 
 * want to block the rest of the system.
 * 
 * To allow a multi-level menu or responders, we can use a Stack that keeps track of active
 * callback functions. When determining if a string should be sent to the port or where
 * to direct input, we can look at the responder at the top of the stack. We'll simply send
 * the input to the responder at the top, and we'll only allow the string to be sent if
 * the sender is the responder at the top. When we want to pass control to a new responder,
 * we simply push it on top of the stack. And when we want to relinquish control of 'exit' the
 * level, we just pop the top responder off the stack.
 * 
 * To implement this, we need:
 *      - A stack that determines the active responder
 *      - An array of registered responders
 * 
 * We'll consider a responder to just be a the callback function that responds to user input,
 * and we'll identify it by its index in the array. The stack will simply contain indices of
 * responders in the array.
 * 
 * When each part of the program initializes, it'll register its corresponding responders with
 * this class, and will receive an index in the array as the return value for each of them.
 * 
 */

#ifndef __PAMSERIAL_H__
#define __PAMSERIAL_H__

class PAMSerial {

};

#endif // __PAMSERIAL_H__