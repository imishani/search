# creates costmap by overlaying a noise image onto existing map
# arguments: input map text file, input image, output text file, desired maximum cost in costmap
# (e.g. corridor.txt perlin.png costmap.txt 500)

from PIL import Image
import sys; args = sys.argv[1:]

def main():
    f_in = open(args[0], 'r')
    costmap_filename = args[2]
    f_out = open("out/" + costmap_filename, 'w')

    THRESHOLD = int(args[3])

    # first 4 lines
    f_out.write(f_in.readline())  # type octile
    HEIGHT = int(f_in.readline().split(' ')[1])
    f_out.write(f'height {HEIGHT}\n')  # height 400
    WIDTH = int(f_in.readline().split(' ')[1])
    f_out.write(f'height {WIDTH}\n')  # width 400
    f_out.write(f_in.readline())  # map

    unresized_img = Image.open(args[1])
    new_size = max(HEIGHT, WIDTH)
    img = unresized_img.resize((new_size, new_size))  # crops input image instead of stretching

    for i in range(HEIGHT):
        for j in range(WIDTH):
            c = f_in.read(1)
            if c == '\n':
                f_out.write('\n')
                continue
            if c == '@':
                num = THRESHOLD
            elif c == 'T':
                num = THRESHOLD//2
            else:
                num = int(img.getpixel((j, i)) / 255 * THRESHOLD * 0.75)
            f_out.write(f'{num}\t')

    f_in.close()
    f_out.close()
    print(f'Created costmap: {costmap_filename}')


if __name__ == '__main__': main()