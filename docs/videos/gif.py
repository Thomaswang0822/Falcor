from moviepy.editor import VideoFileClip
import os

os.chdir("./docs/videos")

def mp4ToGif(name: str):
    mp4_file = name + ".mp4"
    gif_file = name + ".gif"

    # Load the video clip
    clip = VideoFileClip(mp4_file)

    # Convert to GIF (optimize fps to reduce size)
    clip.write_gif(gif_file, fps=30)  # Adjust fps if needed


if __name__ == "__main__":
    mp4ToGif("ReSTIRPT")
    mp4ToGif("ReSTIRPT-denoiser")
    mp4ToGif("PT-denoiser")
