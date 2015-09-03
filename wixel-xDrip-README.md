xDrip for Wixel
=================

### What is it?
This project can be loaded onto a wixel, it will catch the wireless signals
sent from a dexcom transmitter, read, break up the different parts, and then send it using UART so it can be
retransmitted through Bluetooth

### How do I use it?
Its easy!
* step 1: Clone the repo!
  * Using Git:
    `git clone https://github.com/StephenBlackWasAlreadyTaken/wixel-xDrip.git`
  * Scared of Git?
    Download the
    [ZIP](https://github.com/StephenBlackWasAlreadyTaken/wixel-xDrip/archive/master.zip) and unpack it!

* step 2: Install the Wixel drivers and software
  * You can find them [here!](http://www.pololu.com/docs/0J46/3)

* step 2.5: Modify the top section of the file `apps/xdrip/xdrip.c` to fit
your needs
  * Likely you will want to enter your actual transmitter id (Should only be numbers and **CAPITAL LETTERS** , and set
`only_listen_to_my_transmitter` to 1.
  * Leaving `status_lights = 1` will blink the yellow light whenever it is searching for a packet (also you may notice a faint red blink when it strobes the radio) Setting it to zero will make it not blink (the red strobe will still happen though)
  * The USB enabled and close options have been removed, instead the wixel will detect its usb state every time it wakes up to scan for packets. This means if you plug in the wixel your computer may not see it for a few minutes (5)   * if for some reason you still cant get your computer to recognize it, touch a wire from RST to GND
  * If you did something crazy and locked up the bootloader, you will need to jump the wixel into BootLoader mode. see
[this](http://www.pololu.com/docs/0J46/5.c) for instructions on getting the
wixel back into bootloader mode.


* step 3: Run the MAKE command to generate a file you can install on your Wixel
  
  ##### ON A MAC
 * If you do not already have sdcc, you will need to do the following (not sure? open a terminal and run `sdcc -v` if you get the error `sdcc: command not found` then you need to install it!
  * Open a new terminal (if you cant find terminal, just search for it in spotlight) then run the following commands (just coppy and paste this whole big ol thing)
  
  `curl -O http://sourceforge.net/projects/sdcc/files/sdcc-macosx/3.4.0/sdcc-3.4.0-universal-apple-macosx.tar.bz2 -L; tar xjf sdcc-3.4.0-universal-apple-macosx.tar.bz2; cd sdcc-3.4.0/bin; sudo cp -r -n * /usr/bin; cd ../share/sdcc;sudo mkdir /usr/share/sdcc;sudo cp -r -n * /usr/share/sdcc;`
    *  it should ask for your password, just type in your macs password and hit enter (Note that you will not see anything while typing the password)
  * Cool, now if you run `sdcc -v` you should see something like `SDCC : mcs51/z80/z180....`
  
 * Now back to our wixel app!
    * In your terminal/command prompt head to the directory where you cloned/unzipped this repo.
    * Run `make`
    * If you dont already have Command line tools installed it will ask you if you want to install them, just say yes and let it do its thing (note, on a mac version earlier than maverics, you will need to download xcode manually in order to install command line tools)
    * If successfull you should see a lot of lines that end something like 
   ```
   Linking apps/xdrip/xdrip.hex
   Packaging apps/xdrip/xdrip.wxl
   packihx: read 413 lines, wrote 779: OK.
   ```
    ##### ON UBUNTU-LINUX (14.04 or later, should also work on Debian Jessie or later)
 * If you don't already have SDCC installed, open a terminal and run the command 'sudo apt-get install sdcc' and follow the instructions on screen (enter password, confirm with 'y', ...). (Not sure if installed?: run 'sdcc -v'. If the result is '/usr/bin/sdcc: No such file or directory', install it.)
 * If it is installed, the version number returned on the command 'sdcc -v' should be at least 3.3.0
 * From now on, you can follow the instructions for MAC starting from "Now back to our wixel app!".
 
  ##### ON WINDOWS
  * Install the wixel development bundle for windows from [here](http://www.pololu.com/file/0J526/wixel-dev-bundle-120127.exe) (you need this in addition to the drivers you downloaded in step 2!!)
  * Open a command prompt (should be under start menu -> programs -> accessories -> Command Prompt)
  * Cd into the folder you downloaded this repo to (ex. `cd C:\Users\<computer name>\Desktop\wixel-xDrip`)
  * now run `make_all.bat` by typing `make_all.bat` into the command prompt
  

  * You should see a lot of lines that end something like 
  ```
  Linking apps/xdrip/xdrip.hex
  Packaging apps/xdrip/xdrip.wxl
  packihx: read 413 lines, wrote 779: OK.
  ```
* step 4: Load the app
  * plug your wixel in to your computer and follow [these steps](http://www.pololu.com/docs/0J46/3.d) to install your app!

* step 5: Done with this part! Now hook it up to something!!!


##### NOTE:
The wixel will transmit the data out over uart using pins p1_6 and p1_7 at a
baud rate of 9600, feel free to change that in the code if you need.


# HUGE PROPS TO:
Adrien De Croy for writing most of this for his Dexterity Prject!

Lorelai for pointing me to all of these great resources and then also allowing me to use large portions of her code.

John Stevens for further improving on Adrians code, which I also then used!!


Wow.. Im begining to think I didnt do anything other than copy and paste these peoples works...


Don Brown over at [dexwatch](http://dexwatch.blogspot.com/) for initially pointing
out that this is a possibility to me!

Ben West, John Costik and Scott Leibrand their awesome work with Nightscout and various other projects!

And all the other awesome people that helped me out and contributed to awesome projects like NightScout


# LINKS
* [Project Site](http://stephenblackwasalreadytaken.github.io/xDrip/)
* [What you will need & Diagrams](https://github.com/StephenBlackWasAlreadyTaken/xDrip/blob/gh-pages/hardware_setup.md)
* [Wixel App](https://github.com/StephenBlackWasAlreadyTaken/wixel-xDrip)
* [Android App](https://github.com/StephenBlackWasAlreadyTaken/xDrip)
