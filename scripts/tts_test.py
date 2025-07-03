import speech_recognition as sr

def list_microphones():
    """Prints a list of available microphones with their device indices."""
    print("Available microphone devices:")
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        print("Device index",index,name)

if __name__ == "__main__":
    list_microphones()