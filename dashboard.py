# Updated version of your web.py with local MP3 playback

from flask import Flask, jsonify, render_template, request
import spotipy
from spotipy.oauth2 import SpotifyOAuth
import threading, time
import lyricsgenius
import serial
import os
from pydub import AudioSegment
import numpy as np

app = Flask(__name__)
gesture_log = []

# USB CDC configuration
serial_port = "/dev/tty.usbmodem11301"  # Adjust as needed
baud_rate = 115200
dac_resolution_bits = 12
dac_max_value = (1 << dac_resolution_bits) - 1
target_sample_rate = 8000  # 8 kHz for UART compatibility

# Folder containing your MP3 files
MP3_FOLDER = os.path.join(os.path.dirname(__file__), 'mp3')

# Spotify setup
sp = spotipy.Spotify(auth_manager=SpotifyOAuth(
    client_id='f5a3e4ad110f4988818478e64c3023cc',
    redirect_uri='http://127.0.0.1:8888/callback',
    scope='user-read-playback-state user-modify-playback-state',
    cache_path='.cache-sara'
))

# Genius lyrics setup
genius = lyricsgenius.Genius("tXzQ4py8kdeUAW25o4c-9jbRsXh01g0FEra1pT2oY6mt136TTftift3366Ago5GI", skip_non_songs=True)

def handle_gesture(gesture):
    print(f"Gesture: {gesture}")
    gesture_log.append(gesture)

    try:
        if gesture == "left":
            sp.pause_playback()
            play("Playback.mp3")
            sp.previous_track()
        elif gesture == "right":
            sp.pause_playback()
            play("Next.mp3")
            sp.next_track()
        elif gesture == "up":
            vol = sp.current_playback()['device']['volume_percent']
            sp.volume(min(100, vol + 10))
        elif gesture == "down":
            vol = sp.current_playback()['device']['volume_percent']
            sp.volume(max(0, vol - 10))
        elif gesture == "start":
            play("Start.mp3")
            sp.start_playback()
        elif gesture == "stop":
            play("Stahp.mp3")
            sp.pause_playback()
    except Exception as e:
        print(f"Spotify command failed: {e}")

def play(song):
    path = os.path.join(MP3_FOLDER, song)
    audio = AudioSegment.from_mp3(path)
    audio = audio.set_frame_rate(target_sample_rate).set_channels(1)
    samples = np.array(audio.get_array_of_samples())

    if audio.sample_width == 1:
        samples = samples.astype(np.uint16) * dac_max_value // 255
    elif audio.sample_width == 2:
        samples = ((samples.astype(np.int32) + 32768) * dac_max_value // 65535).astype(np.uint16)
    else:
        raise ValueError("Unsupported sample width")

    with serial.Serial(serial_port, baud_rate) as ser:
        delay = 1.0 / (target_sample_rate + 3000)
        for sample in samples:
            ser.write(bytes([(sample >> 8) & 0x0F, sample & 0xFF]))
            time.sleep(delay)

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
                import re
                cleaned_lyrics = re.sub(r'\n?You might also like.*', '', raw_lyrics, flags=re.DOTALL)
                cleaned_lyrics = re.sub(r'\n?Embed.*', '', cleaned_lyrics, flags=re.DOTALL)
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
    return jsonify(gesture_log[-10:])

@app.route('/gesture_control', methods=['POST'])
def gesture_control():
    data = request.get_json()
    gesture = data.get('gesture')
    if gesture:
        handle_gesture(gesture)
        return jsonify({"status": "ok", "gesture": gesture})
    return jsonify({"error": "No gesture provided"}), 400

@app.route('/local_tracks')
def list_local_tracks():
    files = [f for f in os.listdir(MP3_FOLDER) if f.endswith('.mp3')]
    return jsonify(files)

@app.route('/play_local_track', methods=['POST'])
def play_local_track():
    data = request.get_json()
    filename = data.get('filename')
    if not filename:
        return jsonify({'status': 'error', 'message': 'No filename provided'}), 400
    if not os.path.isfile(os.path.join(MP3_FOLDER, filename)):
        return jsonify({'status': 'error', 'message': 'File not found'}), 404
    threading.Thread(target=play, args=(filename,), daemon=True).start()
    return jsonify({'status': 'playing', 'file': filename})

def serial_listener():
    try:
        with serial.Serial('/dev/tty.usbmodem0007736293431', 115200, timeout=1) as ser:
            print("[Serial connected]")
            buffer = ""
            while True:
                char = ser.read().decode(errors='ignore')
                if char == '\n' or char == '\r':
                    gesture = buffer.strip()
                    if gesture:
                        handle_gesture(gesture)
                    buffer = ""
                else:
                    buffer += char
    except serial.SerialException as e:
        print(f"Serial port error: {e}")

threading.Thread(target=serial_listener, daemon=True).start()

if __name__ == '__main__':
    app.run(debug=True)
