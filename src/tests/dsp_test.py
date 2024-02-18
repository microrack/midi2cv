import subprocess
import os
import sys
import matplotlib.pyplot as plt
import numpy as np

def build_dsp_module():
    cmd = [
        "g++",
        "-O3",
        "-Wall",
        "-shared",
        "-std=c++11",
        "-fPIC",
        "-DPYBIND11_TESTING_MODULE",
        "`python3 -m pybind11 --includes`",
        "dsp_pybind11.cpp",
        "-o",
        "dsp`python3-config --extension-suffix`",
        "-I../",
    ]
    print("Building dsp module...")
    subprocess.check_call(" ".join(cmd), shell=True)

# Check if the dsp module is already built
module_name = "dsp"
built_module_path = module_name + "`python3-config --extension-suffix`"
if not os.path.exists(built_module_path):
    build_dsp_module()

# Now you can import the built module
import dsp

def test_clock(clock_type):
    dsp_instance = dsp.Dsp()
    dsp_instance.set_clock_source(clock_type)
    triggerSync = []
    pulseSync = []
    midiClockTrigger = []

    TOTAL_SAMPLES = 15000

    midiInClock = np.zeros(TOTAL_SAMPLES).astype(int)
    midiInClock[200::10] = 1

    # 3 ms pulses
    syncInClock = np.zeros(TOTAL_SAMPLES).astype(int)
    syncInClock[200:900:50] = 1
    syncInClock[201:900:50] = 1
    syncInClock[202:900:50] = 1

    syncInClock[900:1500:53] = 1
    syncInClock[901:1500:53] = 1
    syncInClock[902:1500:53] = 1

    syncInClock[1500::122] = 1
    syncInClock[1501::122] = 1
    syncInClock[1502::122] = 1

    # one second
    tempo = 120
    for i in range(TOTAL_SAMPLES):
        if i > 700:
            tempo = 80
        input_dict = {
            "tempo": tempo,
            "syncIn": syncInClock[i],
            "midiClockTrigger": midiInClock[i],
            "glide": 0.5,
            "mmidiNoteCv" : [0.0],
            "noteOnTrigger" : [0],
            "noteOffTrigger" : [0],
            "pinCv" : [0.0],
            "pinGate" : [0.0],
            "pinMod" : [0.0]
        }

        output_dict = dsp_instance.tick(input_dict)
        triggerSync += [output_dict["triggerSync"]]
        pulseSync += [output_dict["pulseSync"]]
        midiClockTrigger += [-output_dict["midiClockTrigger"]]

    return triggerSync, pulseSync, midiClockTrigger

# Example usage of the dsp module
def main():
    triggerSyncI, pulseSyncI, midiClockTriggerI = test_clock(dsp.ClockSource.Internal)
    triggerSyncS, pulseSyncS, midiClockTriggerS = test_clock(dsp.ClockSource.SyncIn)
    triggerSyncM, pulseSyncM, midiClockTriggerM = test_clock(dsp.ClockSource.MidiIn)



    fig, axs = plt.subplots(3, 1)
    axs[0].plot(triggerSyncI)
    axs[0].plot(midiClockTriggerI)
    axs[0].set_title('Internal')
    axs[1].plot(triggerSyncS)
    axs[1].plot(midiClockTriggerS)
    axs[1].plot(pulseSyncS)
    axs[1].set_title('SyncIn')
    axs[2].plot(triggerSyncM)
    axs[2].plot(midiClockTriggerM)
    axs[2].set_title('MidiIn')

    plt.show()

if __name__ == "__main__":
    main()
