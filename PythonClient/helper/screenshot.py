import setup_path 
import airsim
import numpy as np

import os
import logging
from PIL import Image


log = logging.getLogger()


class screenshotHelper:
    _DEFAULT_FOLDER = 'D:\\airsim_v1.2.0\\screenshot\\'

    def __init__(self, drone_name, client, image_folder=''):
        self.image_id = 1
        self.drone_name = drone_name
        self.client = client
        if not image_folder:
            image_folder = self._DEFAULT_FOLDER
        image_folder += '{}\\'.format(self.drone_name)
        if not os.path.isdir(image_folder):
            try:
                os.makedirs(image_folder)
            except OSError:
                log.error('cannot create dir: {}'.format(image_folder))
        self.image_folder = image_folder
        log.info('save image to: {}'.format(self.image_folder))

    def do_screenshot(self, is_display=False):
        """ get camera images from the car
        Returns:
            image_paths (list): filepaths of the screenshots
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene),
            # airsim.ImageRequest("bottom_center", airsim.ImageType.Scene),
        ], vehicle_name=self.drone_name)
        log.debug('Retrieved images: %d' % len(responses))
        filename_prefix = self.image_folder + "{}-".format(self.image_id)
        self.image_id += 1
        image_paths = []
        for idx, response in enumerate(responses):
            filename = filename_prefix + str(idx)
            if response.compress: #png format
                log.debug('image type {}, size {}'.format(
                    response.image_type, len(response.image_data_uint8)))
                filename = os.path.normpath(filename + '.png')
                airsim.write_file(filename, response.image_data_uint8)
            else:
                log.error('error: image format not support')
            image_paths.append(filename)
        if is_display:
            self.display(image_paths)
        return image_paths

    @staticmethod
    def display(filepath_list):
        images = [Image.open(file) for file in filepath_list]
        padding = 5
        widths, heights = zip(*(i.size for i in images))
        total_width = sum(widths) + padding * len(images)
        max_height = max(heights)
        new_im = Image.new('RGB', (total_width, max_height))
        x_offset = 0
        for im in images:
            new_im.paste(im, (x_offset, 0))
            x_offset += im.size[0] + padding
        new_im.show()