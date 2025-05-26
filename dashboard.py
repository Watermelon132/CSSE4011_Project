from flask import Flask, jsonify, render_template
import spotipy
from spotipy.oauth2 import SpotifyOAuth
import threading, time
import lyricsgenius
import serial

app = Flask(__name__)
gesture_log = []

# Spotify setup
sp = spotipy.Spotify(auth_manager=SpotifyOAuth(
    client_id= 'f5a3e4ad110f4988818478e64c3023cc',
    client_secret='d0ba0612f02944868cfaa730616596a8',
    redirect_uri='http://127.0.0.1:8888/callback',
    scope='user-read-playback-state user-modify-playback-state',
    cache_path='.cache-sara'
))

# Genius lyrics setup (optional)
genius = lyricsgenius.Genius("tXzQ4py8kdeUAW25o4c-9jbRsXh01g0FEra1pT2oY6mt136TTftift3366Ago5GI", skip_non_songs=True)

def handle_gesture(gesture):
    print(f"Gesture: {gesture}")
    gesture_log.append(gesture)

    try:
        if gesture == "left":
            sp.previous_track()
        elif gesture == "right":
            sp.next_track()
        elif gesture == "up":
            vol = sp.current_playback()['device']['volume_percent']
            sp.volume(min(100, vol + 10))
        elif gesture == "down":
            vol = sp.current_playback()['device']['volume_percent']
            sp.volume(max(0, vol - 10))
        elif gesture == "start":
            sp.start_playback()
        elif gesture == "stop":
            sp.pause_playback()
    except Exception as e:
        print(f"Spotify command failed: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/current_track')
def current_track():
    try:
        playback = sp.current_playback()
        if not playback or not playback['item']:
            return jsonify({"error": "No track playing"})
        item = playback['item']
        song = {
            "name": item['name'],
            "artist": item['artists'][0]['name'],
            "album_cover": item['album']['images'][0]['url']
        }

        try:
            lyrics_obj = genius.search_song(item['name'], item['artists'][0]['name'])
            if lyrics_obj:
                raw_lyrics = lyrics_obj.lyrics

                # === CLEANING SECTION ===
                import re
                # Remove "You might also like" and "Embed" sections
                cleaned_lyrics = re.sub(r'\n?You might also like.*', '', raw_lyrics, flags=re.DOTALL)
                cleaned_lyrics = re.sub(r'\n?Embed.*', '', cleaned_lyrics, flags=re.DOTALL)

                # Start from first [Intro] or [Verse] if available
                start_match = re.search(r'\[(Intro|Verse).*?\]', cleaned_lyrics)
                if start_match:
                    cleaned_lyrics = cleaned_lyrics[start_match.start():]

                song["lyrics"] = cleaned_lyrics.strip()
            else:
                song["lyrics"] = "Lyrics not found."
        except Exception as e:
            print(f"Lyrics fetch error: {e}")
            song["lyrics"] = "Lyrics unavailable."

        return jsonify(song)
    except Exception as e:
        return jsonify({"error": str(e)})

@app.route('/gestures')
def get_gestures():
    return jsonify(gesture_log[-10:])  # last 10 gestures

# # Simulate gestures for demo
# def simulate_gestures():
#     gestures = ["start", "right", "left", "up", "up", "down","down", "stop"]
#     delays = [2, 5, 5, 1, 1, 1, 1, 2]
#     for g, d in zip(gestures, delays):
#         handle_gesture(g)
#         time.sleep(d)

# Serial thread
def serial_listener():
    try:
        with serial.Serial('/dev/tty.usbmodem0010502671701', 115200, timeout=1) as ser:
            print("[Serial connected]")
            buffer = ""

            while True:
                char = ser.read().decode(errors='ignore')  # read one character at a time
                if char == '\n' or char == '\r':           # treat newline or carriage return as "end of gesture"
                    gesture = buffer.strip()
                    if gesture:
                        handle_gesture(gesture)
                        buffer = ""  # reset for next word
                else:
                    buffer += char
    except serial.SerialException as e:
        print(f"Serial port error: {e}")

threading.Thread(target=serial_listener, daemon=True).start()

if __name__ == '__main__':
    app.run(debug=True)
