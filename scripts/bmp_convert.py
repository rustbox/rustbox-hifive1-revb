import click
from PIL import Image

@click.group()
def cli():
    pass

@cli.command()
@click.option("--raw", type=click.Path(dir_okay=False), required=False)
@click.argument("bmp", type=click.Path(exists=True, dir_okay=False))
def convert(raw, bmp):
    click.echo(bmp)
    with Image.open(bmp) as image:
        click.echo(image.getbands())
        click.echo(image.mode)
        click.echo(image.size)

        color_data = []
        for (r, g, b) in image.getdata():
            red = into_2bits(r)
            green = into_2bits(g)
            blue = into_2bits(b)
            
            color_byte = small_colors_into_byte(red, green, blue)
            color_data.append(color_byte)
        
        if raw:
            with open(raw, 'wb') as f:
                f.write(bytes(color_data))

        click.echo("{}".format(color_data))




def into_2bits(value):
    if value >=0 and value < 64:
        return 0
    if value >= 64 and value < 128:
        return 1
    if value >= 128 and value < 192:
        return 2
    if value >= 192:
        return 3

def small_colors_into_byte(r, g, b):
    return b << 4 | g << 2 | r


if __name__ == "__main__":
    cli()
