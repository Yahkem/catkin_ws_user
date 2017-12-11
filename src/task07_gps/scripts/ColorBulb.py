class ColorBulb(object):
    def __init__(self, name, real_coords=None, lower_bound=None, upper_bound=None):
        if real_coords is None:
            self.__init__(name['name'], name['real'], name['lower_bound'], name['upper_bound'])
            self.x_img = name['x']
            self.y_img = name['y']
        else:
            self.name = name
            self.lower_bound = lower_bound
            self.upper_bound = upper_bound
            self.x_img = -1
            self.y_img = -1
            self.x_real = real_coords[0]
            self.y_real = real_coords[1]

    def img_coords_str(self):
        return "%s bulb has image coordinates = [%s,%s]px" % (self.name, self.x_img, self.y_img)

    def save_serializable(self):
        return {
            'name': self.name,
            'x': self.x_img,
            'y': self.y_img,
            'real': [self.x_real, self.y_real],
            'lower_bound': self.lower_bound,
            'upper_bound': self.upper_bound,
        }