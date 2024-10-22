from PIL import Image
import os

os.chdir("./docs")

def combine2x2():
    # List of image file paths
    image_paths = [
        "images/none.png",
        "images/sp.png",
        "images/tmp.png",
        "images/full.png"
    ]

    # Open images
    images = [Image.open(img) for img in image_paths]

    # Calculate the size of the 2x2 grid
    max_width = max(img.width for img in images)
    max_height = max(img.height for img in images)

    # Create a new blank image with the calculated size
    combined_image = Image.new("RGB", (max_width * 2, max_height * 2))

    # Paste images into the 2x2 grid
    positions = [(0, 0), (max_width, 0), (0, max_height), (max_width, max_height)]

    for img, pos in zip(images, positions):
        combined_image.paste(img, pos)

    # Save the combined image
    combined_image.save("images/sptmp_2x2.png")

def combine1x3():
    # List of image file paths
    image_paths = [
        "images/PT9frames.png",
        "images/full.png",
        "images/fullD.png"
    ]

    # Open images
    images = [Image.open(img) for img in image_paths]

    # Calculate total width and max height for a 1x3 grid
    total_width = sum(img.width for img in images)
    max_height = max(img.height for img in images)

    # Create a new blank image with the calculated size
    combined_image = Image.new("RGB", (total_width, max_height))

    # Paste images side-by-side into the new image
    x_offset = 0
    for img in images:
        combined_image.paste(img, (x_offset, 0))
        x_offset += img.width

    # Save the combined image
    combined_image.save("images/eqtimeComp.png")

def combine1x2():
    # List of image file paths
    image_paths = [
        "images/rc.png",
        "images/full.png"
    ]

    # Open images
    images = [Image.open(img) for img in image_paths]

    # Calculate total width and max height for a 1x3 grid
    total_width = sum(img.width for img in images)
    max_height = max(img.height for img in images)

    # Create a new blank image with the calculated size
    combined_image = Image.new("RGB", (total_width, max_height))

    # Paste images side-by-side into the new image
    x_offset = 0
    for img in images:
        combined_image.paste(img, (x_offset, 0))
        x_offset += img.width

    # Save the combined image
    combined_image.save("images/shiftComp.png")

def combineRef():
    # List of image file paths
    image_paths = [
        "images/accRef.png",
        "images/accReSTIR.png"
    ]

    # Open images
    images = [Image.open(img) for img in image_paths]

    # Calculate total width and max height for a 1x3 grid
    total_width = sum(img.width for img in images)
    max_height = max(img.height for img in images)

    # Create a new blank image with the calculated size
    combined_image = Image.new("RGB", (total_width, max_height))

    # Paste images side-by-side into the new image
    x_offset = 0
    for img in images:
        combined_image.paste(img, (x_offset, 0))
        x_offset += img.width

    # Save the combined image
    combined_image.save("images/accComp.png")

if __name__ == "__main__":
    combine2x2()
    combine1x3()
    combine1x2()
    combineRef()
