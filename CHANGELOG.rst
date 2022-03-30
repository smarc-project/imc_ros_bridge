^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imc_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2022-03-30)
------------------
* Update README.md
* Add files via upload
* Create vehicles.md
* Update release.yml
* Merge pull request `#26 <https://github.com/smarc-project/imc_ros_bridge/issues/26>`_ from KKalem/noetic-devel
  Added CoverArea maneuver
* Added CoverArea maneuver
* Merge pull request `#25 <https://github.com/smarc-project/imc_ros_bridge/issues/25>`_ from KKalem/noetic-devel
  Changed lolo's defaults to -30m and 200rpm for goto/sample maneuvers
* Changed lolo's defaults to -30m and 200rpm for goto/sample maneuvers
* Merge pull request `#24 <https://github.com/smarc-project/imc_ros_bridge/issues/24>`_ from KKalem/noetic-devel
  New "Sample" maneuver and imc_id semantics changes
* Added sample maneuver to lolo, removed unhandled maneuvers, added lolos picture
* Added handling for the SAMPLE maneuver, changed the imc_id imc_src semantics to match Neptus, started to ignore tcp_callback for planDB messages that were spammy, added new fields to Maneuver.msg for the SAMPLE maneuver
* Added SAMPLE maneuver to SAM, added SAM's picture to vehicle defs
* Merge pull request `#23 <https://github.com/smarc-project/imc_ros_bridge/issues/23>`_ from KKalem/noetic-devel
  Updated instructions and files for the new Neptus version.
* Removed unused files and renamed the folder for easier copying
* Updated README for the newer version of Neptus
* Update release.yml
* Merge pull request `#22 <https://github.com/smarc-project/imc_ros_bridge/issues/22>`_ from nilsbore/test_release
  Release building
* Create release.yml
* Added install of targets
* Merge pull request `#21 <https://github.com/smarc-project/imc_ros_bridge/issues/21>`_ from smarc-project/lab_test
  Put node in imc namespace
* Added install rules
* Put node in imc namespace
* Merge pull request `#20 <https://github.com/smarc-project/imc_ros_bridge/issues/20>`_ from smarc-project/nilsbore-patch-1
  Update main.yml
* Update main.yml
* Update README.md
* Update package.xml
* Update package.xml
* Create main.yml
* Merge pull request `#18 <https://github.com/smarc-project/imc_ros_bridge/issues/18>`_ from KKalem/master
  Slight improvement to the md5 situation.
* Changed md5 calcs to be 1-to-1 to java version. md5s still not matching with neptus, but this is a better start than before
* Corrected the md5 format. Removed some noise
* Merge pull request `#17 <https://github.com/smarc-project/imc_ros_bridge/issues/17>`_ from KKalem/master
  Added plandbinformation and plandbstate messages for better neptus in…
* Added plandbinformation and plandbstate messages for better neptus integration. Somehow the md5 we calculate here is different than the one Neptus calculates and compares, will fix later.
* Merge pull request `#16 <https://github.com/smarc-project/imc_ros_bridge/issues/16>`_ from KKalem/master
  Added ros to imc planDB message xlation so we can answer neptus
* Added ros to imc planDB message xlation so we can answer neptus's requests
* Merge pull request `#15 <https://github.com/smarc-project/imc_ros_bridge/issues/15>`_ from KKalem/master
  Made multiple bridges/vehicles possible.
* Added new imc_src argument to udp_link. imc messages have src_ent to separate different vehicles of the same type and id. imc_src is passed on to src_ent of every message
* Merge pull request `#14 <https://github.com/smarc-project/imc_ros_bridge/issues/14>`_ from KKalem/master
  Added new launch file, cleanup
* Added new launch file, bridge_sam which has sam-specific defaults and calls bridge.launch with those, cleanup
* Merge pull request `#13 <https://github.com/smarc-project/imc_ros_bridge/issues/13>`_ from KKalem/master
  Added proper planDB message. At least the GOTO maneuvers are in there now.
* Merge branch 'master' of https://github.com/smarc-project/imc_ros_bridge
* Added new PlanDB message _and all the billion sub-messages it needed\_
* Merge pull request `#10 <https://github.com/smarc-project/imc_ros_bridge/issues/10>`_ from NiklasRolleberg/master
  rosparam for ip of bridge and netpus + more conversions + compilation fix on jetson
* Merge pull request `#2 <https://github.com/smarc-project/imc_ros_bridge/issues/2>`_ from NiklasRolleberg/master
  merge niklas initial
* Added RemoteState conversion
* Merge branch 'master' into master
* Added parameter for chaging the name of the node and increased the precission of floats in missions to get better accuracy of waypoints
* Merge pull request `#12 <https://github.com/smarc-project/imc_ros_bridge/issues/12>`_ from KKalem/master
  Added namespacing to bridge.launch.
* Added the old Estimated state conversion again
* Merge branch 'lidingo_changes'
* Added namepsacing to bridge launch file. default to sam
* Merge pull request `#11 <https://github.com/smarc-project/imc_ros_bridge/issues/11>`_ from KKalem/master
  imc_id changes so that they are consistent. Also small README update.
* Merge branch 'master' into master
* Update README.md
* Merge branch 'master' of https://github.com/smarc-project/imc_ros_bridge
* Changed default imc_ids to be consistent
* Merge pull request `#9 <https://github.com/smarc-project/imc_ros_bridge/issues/9>`_ from KKalem/master
  Updated README with JRE installation clarification for ubuntu 18
* changed namespace
* removed logging
* compiling on jetson
* Added a few new conversions
* removed a line
* changed name of parameters related to ip and port
* Added important note to README
* added a simple way of logging and changed Estimatedstate converter
* Removed namespacing from the bridge in favor of the launch file namespacing method
* Update README with more instructions for Ubuntu 18
* Updated README with JRE installation clarification for ubuntu 18
* Added sonardata message and increased send buffer size to be able to send bigger messages
* changed to x=lat & y = lon in pose message to fit better with the NED frame
* added rosparam for setting client address (address of computer running neptus)
* added missing dependency
* Added vehicle state message
* Changed so heading=0 is north when using data from lolo
* Merge remote-tracking branch 'origin/master'
* Fixed neptus heading conversion
* Merge pull request `#8 <https://github.com/smarc-project/imc_ros_bridge/issues/8>`_ from KKalem/master
  Updated readme with a link to the bts_tutorial as an example
* Updated readme with a link to the bts_tutorial as an example use of the bridge
* Merge pull request `#7 <https://github.com/smarc-project/imc_ros_bridge/issues/7>`_ from KKalem/master
  Updated README
* Updated readme, couple lines
* Updated readme to be more generic with respect to the vehicle used.
* Update README.md
* Merge pull request `#6 <https://github.com/smarc-project/imc_ros_bridge/issues/6>`_ from KKalem/master
  Changed EstimatedState to Pose from NavSatFix to add orientation to N…
* added tf2 stuff to cmake file
* Changed EstimatedState to Pose from NavSatFix to add orientation to Neptus viz
* Merge pull request `#5 <https://github.com/smarc-project/imc_ros_bridge/issues/5>`_ from KKalem/master
  Added lolo files for neptus
* Update README.md
* added lolos neptus stuff
* Update README.md
* Merge pull request `#4 <https://github.com/smarc-project/imc_ros_bridge/issues/4>`_ from KKalem/master
  imc_id modifications and plumbing, put ros topic names under system namespace, added smarc marketing
* updated readme. added smarc propoganda in the form of a generic vehicle called imc ros bridge to neptus. bridge id and vehicle id must match for console to send stuff
* Now we are passing through the rosparam imc_id to imc message fields src and src_ent, still trying to get 2 bridges to work at the same time
* Changed the src field of the UDP link stuff to 4, which seems to be unused in IMC as of now. Now the bridge shows up in Neptus as whatever system_name we are using.
* Parameter routing fixes, also made it so topics in ros are also created under system_name/...
* Merge pull request `#1 <https://github.com/smarc-project/imc_ros_bridge/issues/1>`_ from smarc-project/master
  Merge smarc-project
* Merge pull request `#3 <https://github.com/smarc-project/imc_ros_bridge/issues/3>`_ from KKalem/master
  fixed topic typo
* fixed topic typo
* Merge pull request `#2 <https://github.com/smarc-project/imc_ros_bridge/issues/2>`_ from KKalem/master
  Updated README
* Updated README
* Update README.md
* Merge pull request `#1 <https://github.com/smarc-project/imc_ros_bridge/issues/1>`_ from smarc-project/add-license-1
  Create LICENSE
* Create LICENSE
* Merge pull request `#4 <https://github.com/smarc-project/imc_ros_bridge/issues/4>`_ from nbore/add_license
  added licenses
* added licenses
* Merge pull request `#3 <https://github.com/smarc-project/imc_ros_bridge/issues/3>`_ from ozero/master
  PlanControl and PlanControlState added
* Added plan control state, feedback from the AUV to Neptus about whats going on with the plan, made msg generation reliable
* Added PlanControl message. Also made a ros msg definiiton for it. This message handles the run and stop buttons in the console
* Merge pull request `#2 <https://github.com/smarc-project/imc_ros_bridge/issues/2>`_ from ozero/master
  Buffer size too small somewhere
* removed extra print
* Added EstimatedState, this updates the location of SAM in neptus console, added sam_files/... these can be added to neptus vehicles in order to see some visuals and later define some properties of SAM. Updated README with basic usage instructions
* Fixed the too small buffer problem in udp_link, now the buffer for receiving is 16Kib. Should be enough for most things i imagine
* added abort and plandb message handling. Large planDB's are not being parsed for some reason, some debugging code in udp_link.cpp left for later
* Not having working template overrides are now a compile error. imc->ros heartbeat working
* Attempting to add imc to ros Heartbeat. Template function is not overwriting the default
* Update README.md
* Update README.md
* Update README.md
* Merge branch 'udp'
* Made an error, corrected
* Update README.md
* Merge pull request `#1 <https://github.com/smarc-project/imc_ros_bridge/issues/1>`_ from nbore/udp
  Switch to udp
* Made it shut down smoothly
* Moved structure to new way, removed tcp link
* ADded stuff
* Basically a working version
* Got the udp link working
* Added the announce stuff as well
* Semi-working udp link
* Added initial udp link
* Merge remote-tracking branch 'origin/master'
* MVP
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Cleaned up a bit
* Merge remote-tracking branch 'origin/master'
* Initial commit
* Added some more stuff, receiving messages seemingly not working, though hard to debug
* Got both ways compiling
* Got the basic version compiling
* Added the essential parts
* Add 'external/imc-cxx/' from commit '9a02b57550887d0bb2f4470c54f791913684e6c0'
  git-subtree-dir: external/imc-cxx
  git-subtree-mainline: a0af98fec9959cbcc08d83e0c9208941f42150af
  git-subtree-split: 9a02b57550887d0bb2f4470c54f791913684e6c0
* Added first things
* IMC v5.4.20 C++ Bindings.
* IMC v5.4.18 C++ Bindings.
* Contributors: Jollerprutt, Niklas, Nils Bore, Ozer, Ozer Ozkahraman, Ricardo Martins, ignaciotb, niklasrolleberg, Özer Özkahraman
