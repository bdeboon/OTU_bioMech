#!/bin/bash
#Code was written April 2019 to support Ubuntu 16.04 LTS

#Install Inkscape
echo "Installing Inkscape"
sudo add-apt-repository ppa:inkscape.dev/stable
sudo apt update
sudo apt install inkscape

#Install LaTeX
echo "Installing LaTeX"
sudo apt-get install texlive-full

#Install Texmaker
echo "Installing Texmaker"
sudo apt-get install texmaker

#Install TexStudio
echo "Installing TexStudio"
sudo add-apt-repository ppa:sunderme/texstudio
sudo apt-get update
sudo apt-get install texstudio

#Install MikTeX
echo "Installing MiKTeX"
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D6BC243565B2087BC3F897C9277A7293F59E4889
echo "deb http://miktex.org/download/ubuntu xenial universe" | sudo tee /etc/apt/sources.list.d/miktex.list
sudo apt-get update
sudo apt-get install miktex

#Install Atom (Programming Environment)
echo "Installing Atom"
sudo add-apt-repository ppa:webupd8team/atom
sudo apt update; sudo apt install atom

#Install Google Sync (Programming Environment)
echo "Installing Google Sync"
sudo add-apt-repository ppa:alessandro-strada/ppa
sudo apt update && sudo apt install google-drive-ocamlfuse

#How to use ocamlfuse:
#$    google-drive-ocamlfuse
#$    mkdir ~/googledrive
#$    google-drive-ocamlfuse ~/googledrive


#Install GNU Octave (Matlab Substitute)
echo "Installing GNU Octave"
sudo apt-add-repository ppa:octave/stable
sudo apt-get update
sudo apt-get install octave
sudo apt-get install liboctave-dev




