from Speck import Speck


def setup(speck:Speck):
    speck.lb_leg.set_position(0, 0, 0)
    return None


def loop():
    cont = False
    return cont


if __name__ == "__main__":
    speck = Speck()  # create an instance of the Speck object to control
    setup(speck)  # perform all necessary setup tasks
    while loop():  # continue performing all functions in the loop function while the output is true
        pass
