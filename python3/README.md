## Processing .rec files using Python3

This Python-template demonstrates how to process .rec files using Python3.

Prerequisites:
* You have a recording file (`.rec`).
* You need to install `libcluon` to transform message specifications to Protobuf (example below is for Ubuntu 18.04 LTS):
```Bash
sudo add-apt-repository ppa:chrberger/libcluon
sudo apt-get update
sudo apt-get install libcluon
```
* You need to install Python, make, and protobuf (example below is for Ubuntu 18.04 LTS):
```Bash
sudo apt-get install --no-install-recommends \
    build-essential \
    python-protobuf \
    python-sysv-ipc \
    python-numpy \
    python-opencv \
    protobuf-compiler
```

---

### Developing and testing the Python3 application on your computer

This template folder contains an example how to use Python3 to process a .rec file.

* Step 1: Clone this repository:
```bash
cd $HOME
git clone https://github.com/chalmers-revere/working-with-rec-files.git
cd working-with-rec-files/python3
```

* Step 2: The Python3 script uses messages from the OpenDLV Standard Message Set.
To use them with Python, just run:
```bash
make
```
This step needs to be repeated whenever you change something in the message specifications.


* Step 3: Run the Pyton3 module:
```bash
python3 printContentFromRecFile.py example.rec
```

