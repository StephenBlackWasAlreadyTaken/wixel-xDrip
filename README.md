DexDrip for Wixel
=================

### What is it?
This project can be loaded onto a wixel, it will catch the wireless signals
sent from a dexcom transmitter, read, break up the different parts, and then send it using UART so it can be
retransmitted through Bluetooth

### How do I use it?
Its easy!
* step 1: Clone the repo!
  * Using Git:
    `git clone https://github.com/StephenBlackWasAlreadyTaken/wixel-DexDrip.git`
  * Scared of Git?
    Download the
    [ZIP](https://github.com/StephenBlackWasAlreadyTaken/wixel-DexDrip/archive/master.zip) and unpack it!

* step 2: Install the Wixel drivers and software
  * You can find them [here!](http://www.pololu.com/docs/0J46/3)

* step 2.5: Modify the top section of the file `apps/dexdrip/dexdrip.c` to fit
your needs
  * Likely you will want to enter your actual transmitter id, and set
`only_listen_to_my_transmitter` to 1.

  * Leaving `usbEnabled` at `0` will allow you to listen for updates from your
dexcom over usb, handy for debugging, also it will let you reload files onto
your wixel with ease.
setting `usbEnabled` to `1` will result in better battery life, but you loose
the ability to listen for updates over usb AND if you want to update the file
on your wixel, you will need to jump the wixel into BootLoader mode. see
[this](http://www.pololu.com/docs/0J46/5.c) for instructions on getting the
wixel back into bootloader mode.


* step 3: Run the MAKE command to generate a file you can install on your Wixel
  * In your terminal/command prompt head to the directory where you
  cloned/unzipped this repo.
  * Run `make`
  * You should see a lot of lines that end something like 
  ```
  Linking apps/dexdrip/dexdrip.hex
  Packaging apps/dexdrip/dexdrip.wxl
  packihx: read 413 lines, wrote 779: OK.
  ```
* step 4: Load the app
  * plug your wixel in to your computer and follow [these steps](http://www.pololu.com/docs/0J46/3.d) to install your app!

* step 5: Done with this part! Now hook it up to something!!!


##### NOTE:
The wixel will transmit the data out over uart using pins p1_6 and p1_7 at a
baud rate of 9600, feel free to change that in the code if you need.

### Whats a dexcom?
Its an amazing CGM (continue glucose monitor) that many Diabetics use to keep
track of their glucose levels in real(ish)time

### Why this then?
The current dexcom has its own proprietary reciever that unfortunately cannot
talk to phones unless using a project like
[NightScout](http://www.nightscout.info/) Which is an awesome project I HIGHLY recomend you check out

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
* [Project Site](http://stephenblackwasalreadytaken.github.io/DexDrip/)
* [What you will need & Diagrams](https://github.com/StephenBlackWasAlreadyTaken/DexDrip/blob/gh-pages/hardware_setup.md)
* [Wixel App](https://github.com/StephenBlackWasAlreadyTaken/wixel-DexDrip)
* [Android App](https://github.com/StephenBlackWasAlreadyTaken/DexDrip)
