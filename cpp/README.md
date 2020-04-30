## Processing .rec files using C++14

This C++14-template demonstrates how to process .rec files.

Prerequisites:
* You have a recording file (`.rec`).
* You need to have a C++ compiler with C++14 support
* You should make for simple compilation (example below is for Ubuntu 18.04 LTS):
```Bash
sudo apt-get install --no-install-recommends \
    build-essential
```

---

### Developing and testing the C++14 application on your computer

This template folder contains an example how to use C++14 to process a .rec file.

* Step 1: Clone this repository:
```bash
cd $HOME
git clone https://github.com/chalmers-revere/working-with-rec-files.git
cd working-with-rec-files/cpp
```

* Step 2: The C++ application uses messages from the OpenDLV Standard Message Set.
To compile the complete example, just run:
```bash
make
```
This step needs to be repeated whenever you change something in the message specifications.


* Step 3: Run the C++ application:
```bash
./printContentFromRecFile example.rec
```

You find more information at [Tutorial: "Getting Started" explaining how to exchange messages between distributed applications (using an online C++ compiler)](https://wandbox.org/permlink/3S1bSOaLakXfdWWZ)
