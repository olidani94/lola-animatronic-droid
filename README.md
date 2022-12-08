# lola-animatronic-droid
This project contains the Arduino code used to control a DIY animatronic version of L0-LA59 (Lola) from Star Wars Obi-Wan Kenobi 

The Youtube video with the build process is: https://www.youtube.com/watch?v=chb6E1pl1ec

List of hardware: 
- 2x micro servos SG90
- Mini MP3 DFPlayer + Speaker
- LM393 microphone module
- Arduino Nano (you can find the pins that I used in the sketch)
- Button switch to trigger evil mode
- RGB leds for the eyes

Software notes:
- I added a test function to test the different hardware components, feel free to uncomment it to test your hardware
- The servo min and max values should be adjusted to your build
- I used a sound detector to trigger responses from Lola but you can remove it
- The sound detector should be adjusted as well using the potentiometer
- I added an evil mode where lola plays anger sounds and switches the lights to red, please feel free to remove it.
- I added a flying mode where lola plays a hovering sound and opens the wings for some time.
- For the sounds I used the DFPlayer library: https://github.com/DFRobot/DFRobotDFPlayerMini
- To configure the DFPlayer module I used the following tutorial: https://www.electroallweb.com/index.php/2020/07/22/modulo-dfplayer-mini-reproductor-mp3-tutorial-completo/
- The tutorial shows the schematic and also recommends the folder structure that i used, which is storing the files in the following format: SD:/MP3/0005.mp3
- I used 18 sounds recorded with the official animatronic Lola version and stored as /MP3/0001.mp3.../MP3/0018.mp3...the order of the songs is a bit confusing, test the script yourself to play the songs that you want. There are some important sounds:
---- The evil mode sounds are 0001.mp3 and 0002.mp3
---- The flying sounds are 0003.mp3 and 0004.mp3
---- The start sound is 0005.mp3
- I have uploaded the sounds to the MP3.zip file of this repo
