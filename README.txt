Put this whole directory at:
  ~/XING_aarch64_SDK_4.1.0.5634/python_bridge

Expected SDK layout:
  ~/XING_aarch64_SDK_4.1.0.5634/include/NokovSDKClient.h
  ~/XING_aarch64_SDK_4.1.0.5634/include/NokovSDKTypes.h
  ~/XING_aarch64_SDK_4.1.0.5634/lib/libnokov_sdk.so

Build:
  cd ~/XING_aarch64_SDK_4.1.0.5634/python_bridge
  ./build.sh

Run:
  python3 nokov_reader.py

If libnokov_sdk.so is not found:
  export LD_LIBRARY_PATH=$HOME/XING_aarch64_SDK_4.1.0.5634/lib:$LD_LIBRARY_PATH

Default server IP in nokov_reader.py:
  192.168.5.6
