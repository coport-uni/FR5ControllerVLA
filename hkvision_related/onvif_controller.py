import logging
import time
from onvif import ONVIFCamera

class ONVIFController:
    def __init__(self, ip, port, user, password):
        self.__cam_ip = ip
        self.__cam_port = port
        self.__cam_user = user
        self.__cam_password = password

    def camera_start(self):
        """
        Creates the connection to the camera using the onvif protocol
        Returns:
        Return the ptz service object and media service object
        """
        mycam = ONVIFCamera(self.__cam_ip, self.__cam_port, self.__cam_user, self.__cam_password)  ## Some cameras use port 8080
        logging.info('Create media service object')
        media = mycam.create_media_service()
        logging.info('Create ptz service object')
        ptz = mycam.create_ptz_service()
        logging.info('Get target profile')
        media_profile = media.GetProfiles()[0]
        logging.info('Camera working!')

        self.mycam = mycam
        self.camera_ptz = ptz
        self.camera_media_profile = media_profile
        self.camera_media = media

        return self.camera_ptz, self.camera_media_profile

    def set_preset(self, preset_name: str):
        """
        The command saves the current device position parameters.
        Args:
            preset_name: Name for preset.
        Returns:
            Return onvif's response.
        """
        presets = ONVIFController.get_preset_complete(self)
        request = self.camera_ptz.create_type('SetPreset')
        request.ProfileToken = self.camera_media_profile.token
        request.PresetName = preset_name
        logging.info('camera_command( set_preset%s) )', preset_name)

        for i, _ in enumerate(presets):
            if str(presets[i].Name) == preset_name:
                logging.warning(
                    'Preset (\'%s\') not created. Preset already exists!', preset_name)
                return None

        ptz_set_preset = self.camera_ptz.SetPreset(request)
        logging.info('Preset (\'%s\') created!', preset_name)
        return ptz_set_preset

    def get_preset(self):
        """
        Operation to request all PTZ presets.
        Returns:
            Returns a list of tuples with the presets.
        """
        ptz_get_presets = ONVIFController.get_preset_complete(self)
        logging.info('camera_command( get_preset() )')

        presets = []
        for i, _ in enumerate(ptz_get_presets):
            presets.append((i, ptz_get_presets[i].Name))
        return presets
    
    def get_preset_complete(self):
        """
        Operation to request all PTZ presets.
        Returns:
            Returns the complete presets Onvif.
        """
        request = self.camera_ptz.create_type('GetPresets')
        request.ProfileToken = self.camera_media_profile.token
        ptz_get_presets = self.camera_ptz.GetPresets(request)

        return ptz_get_presets
        
    def remove_preset(self, preset_name: str):
        """
        Operation to remove a PTZ preset.
        Args:
            preset_name: Preset name.
        Returns:
            Return onvif's response.
        """
        presets = ONVIFController.get_preset_complete(self)
        request = self.camera_ptz.create_type('RemovePreset')
        request.ProfileToken = self.camera_media_profile.token
        logging.info('camera_command( remove_preset(%s) )', preset_name)
        for i, _ in enumerate(presets):
            if str(presets[i].Name) == preset_name:
                request.PresetToken = presets[i].token
                ptz_remove_preset = self.camera_ptz.RemovePreset(request)
                logging.info('Preset (\'%s\') removed!', preset_name)
                return ptz_remove_preset
        logging.warning("Preset (\'%s\') not found!", preset_name)
        return None

    def go_to_preset(self, preset_position: str):
        """
        Operation to go to a saved preset position.
        Args:
            preset_position: preset name.
        Returns:
            Return onvif's response.
        """
        presets = ONVIFController.get_preset_complete(self)
        request = self.camera_ptz.create_type('GotoPreset')
        request.ProfileToken = self.camera_media_profile.token
        logging.info('camera_command( go_to_preset(%s) )', preset_position)
        for i, _ in enumerate(presets):
            str1 = str(presets[i].Name)
            if str1 == preset_position:
                request.PresetToken = presets[i].token
                resp = self.camera_ptz.GotoPreset(request)
                logging.info("Goes to (\'%s\')", preset_position)
                return resp
        logging.warning("Preset (\'%s\') not found!", preset_position)
        return None

if __name__ == "__main__":
    ptz_cam_218 = ONVIFController("192.168.1.218", 218, "admin", "peal2024")
    ptz_cam_218.camera_start()

    print(ptz_cam_218.get_preset())

    for i in range(5):
        ptz_cam_218.go_to_preset("Preset 1")
        time.sleep(2)

        ptz_cam_218.go_to_preset("Preset 2")
        time.sleep(2)