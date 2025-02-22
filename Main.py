from Speck import Speck


def setup(speck: Speck):
    speck.lb_leg.set_position(0, 100, 0)
    return None


def loop(speck: Speck):
    cont = True  # by default, the loop will continue unless this variable is set to False
    return cont


if __name__ == "__main__":
    speck = Speck()  # create an instance of the Speck object to control
    setup(speck)  # perform all necessary setup tasks
    while loop(speck):  # continue performing all functions in the loop function while the output is true
        pass
