#/bin/sh

rm -rf build/
mkdir -p build/
cd build/

#sip -c ../build/ ../neblina_communication.sip
python ../configure.py
make