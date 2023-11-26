# creates costmap from image
# arguments: desired height of costmap, desired width, input image, output text file
# (e.g. 400 400 perlin.png costmap_simple.txt)

from PIL import Image
import sys; args = sys.argv[1:]

def main():
    HEIGHT, WIDTH = int(args[0]), int(args[1])
    unresized_img = Image.open(args[2])
    new_size = max(HEIGHT, WIDTH)
    img = unresized_img.resize((new_size, new_size))  # crops input image instead of stretching

    costmap_filename = args[3]
    f_out = open('out/' + costmap_filename, 'w')
    f_out.write(f'type octile\nheight {HEIGHT}\nwidth {WIDTH}\nmap\n')

    for i in range(HEIGHT):
        for j in range(WIDTH):
            num = int(img.getpixel((j, i)) * 1.5)
            f_out.write(f'{num}\t')
        f_out.write('\n')

    f_out.close()
    print(f'Created costmap: {costmap_filename}')


if __name__ == '__main__': main()