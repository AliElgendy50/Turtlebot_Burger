#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pyttsx3
import time

def initialize_tts_engine():
    try:
        engine = pyttsx3.init(driverName='espeak')
        return engine
    except Exception as e:
        rospy.logerr(f"Failed to initialize espeak driver: {e}")
    
    try:
        engine = pyttsx3.init(driverName='nsss')
        return engine
    except Exception as e:
        rospy.logerr(f"Failed to initialize nsss driver: {e}")
    
    try:
        engine = pyttsx3.init(driverName='sapi5')
        return engine
    except Exception as e:
        rospy.logerr(f"Failed to initialize sapi5 driver: {e}")

    rospy.logerr("All attempts to initialize pyttsx3 failed.")
    return None

def description_callback(statues_name):
    # Initialize text-to-speech engine
    engine = initialize_tts_engine()
    if not engine:
        rospy.logerr("Text-to-speech engine initialization failed. Cannot proceed with callback.")
        return

    # King Tutankhamun description
    if statues_name.data == 'King-Tut':
        engine.say("Welcome, ladies and gentlemen, to the captivating world of Tutankhamun.")
        time.sleep(4)
        engine.say("The legendary 'Boy King' of ancient Egypt.")
        time.sleep(2)
        engine.say("Tutankhamun, born circa 1341 BC.")
        time.sleep(2)
        engine.say("He ascended the throne at the tender age of nine and ruled for a short but intriguing reign.")
        time.sleep(4)
        engine.say("His tomb, discovered nearly intact in the Valley of the Kings in 1922 by Howard Carter.")
        time.sleep(4)
        engine.say("Unveiled treasures that have captured the imagination of the world ever since.")
        time.sleep(2)
        engine.say("Tutankhamun remains a symbol of Egypt's rich history and mysteries waiting to be unraveled.")

    # Queen Nefertiti description
    elif statues_name.data == 'Nefertiti':
        engine.say("To the enchanting tale of Nefertiti, the illustrious Queen of Egypt.")
        time.sleep(4)
        engine.say("Revered for her unparalleled beauty and grace, Nefertiti graced the courts of ancient Egypt.")
        time.sleep(2)
        engine.say("Alongside her husband, Pharaoh Akhenaten, during the intriguing period of the Amarna era.")
        time.sleep(4)
        engine.say("Her iconic bust, discovered in 1912, captivates with its timeless elegance, symbolizing not just her physical allure,")
        time.sleep(4)
        engine.say("But also her powerful influence as a consort and perhaps even a co-regent.")
        time.sleep(2)
        engine.say("Join me as we journey through the life and legacy of this enigmatic queen.")
        time.sleep(2)
        engine.say("Whose name means 'the beautiful one has come.'")

    # Frederic Chopin description
    elif statues_name.data == 'Chopin':
        engine.say("To the musical world of Frédéric Chopin, the renowned Polish composer and virtuoso pianist.")
        time.sleep(4)
        engine.say("Born in 1810, Chopin's compositions, such as his haunting nocturnes and passionate ballades, continue to captivate audiences worldwide.")
        time.sleep(2)
        engine.say("His innovative style and emotive melodies earned him the title 'Poet of the Piano.'")
        time.sleep(4)
        engine.say("Join me as we explore the life and legacy of this musical genius,")
        time.sleep(2)
        engine.say("Whose works resonate with timeless beauty and profound emotion,")
        time.sleep(2)
        engine.say("Enriching the soul of music lovers for generations to come.")
        time.sleep(4)

    # Chinese Fu Dog description
    elif statues_name.data == 'Chinese Fu Dog':
        engine.say("Step into our museum and encounter the regal presence of a Chinese Fu Dog statue.")
        time.sleep(4)
        engine.say("With its guardian stance and intricate detailing,")
        time.sleep(2)
        engine.say("It embodies ancient myths and cultural significance.")
        time.sleep(2)
        engine.say("Crafted with precision, it stands as a symbol of protection and prosperity,")
        time.sleep(4)
        engine.say("Inviting you to explore the richness of Chinese tradition and heritage.")

    engine.runAndWait()

def main():
    rospy.init_node('description_node', anonymous=True)
    rospy.Subscriber('/object_detection/class', String, description_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
