import cv2
import os

video_path = 'crash.MP4'

# Start and end times in seconds
start_time = 5
end_time = 10

# Output folder for frames
output_folder = 'frames_output'
os.makedirs(output_folder, exist_ok=True)

# Open the video file
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
  print('Error opening the video file!')
  exit()

# Get video information
fps = int(cap.get(cv2.CAP_PROP_FPS))  # Frames per second
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
duration = frame_count / fps  # Total video duration in seconds

print(f'Video duration: {duration:.2f} seconds, FPS: {fps}')

# Convert times to frame numbers
start_frame = int(start_time * fps)
end_frame = int(end_time * fps)

# Check if times are valid
if start_frame >= frame_count or end_frame > frame_count:
  print('The specified times exceed the video duration!')
  cap.release()
  exit()

# Move to the starting frame
cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

frame_number = start_frame
while frame_number <= end_frame:
    ret, frame = cap.read()
    if not ret:
        print(f'Error reading frame {frame_number}')
        break

    # Save the frame as an image
    output_path = os.path.join(output_folder, f'frame_{frame_number:06d}.png')
    cv2.imwrite(output_path, frame)
    print(f'Frame {frame_number} saved to {output_path}')

    frame_number += 1

# Release video file
cap.release()
print(f'Frames from {start_time}s to {end_time}s saved in '{output_folder}'.')
