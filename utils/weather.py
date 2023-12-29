import carla


def get_foggy(w):
    weather = carla.WeatherParameters(cloudiness=w.cloudiness,
                                      precipitation=w.precipitation,
                                      precipitation_deposits=w.precipitation_deposits,
                                      wind_intensity=w.wind_intensity,
                                      sun_azimuth_angle=w.sun_azimuth_angle,
                                      sun_altitude_angle=w.sun_altitude_angle,
                                      dust_storm=w.dust_storm,
                                      mie_scattering_scale=w.mie_scattering_scale,
                                      rayleigh_scattering_scale=w.rayleigh_scattering_scale,
                                      scattering_intensity=w.scattering_intensity,
                                      wetness=w.wetness
                                      )
    weather.fog_density = 50
    weather.fog_distance = 20.0
    weather.fog_falloff = 2.0
    return weather


weather_presets = [
    ("ClearNoon", carla.WeatherParameters.ClearNoon),
    ("CloudyNoon", carla.WeatherParameters.CloudyNoon),
    ("WetNoon", carla.WeatherParameters.WetNoon),
    ("WetCloudyNoon", carla.WeatherParameters.WetCloudyNoon),
    ("MidRainyNoon", carla.WeatherParameters.MidRainyNoon),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
    ("SoftRainNoon", carla.WeatherParameters.SoftRainNoon),
    ("ClearSunset", carla.WeatherParameters.ClearSunset),
    ("CloudySunset", carla.WeatherParameters.CloudySunset),
    ("WetSunset", carla.WeatherParameters.WetSunset),
    ("WetCloudySunset", carla.WeatherParameters.WetCloudySunset),
    ("MidRainSunset", carla.WeatherParameters.MidRainSunset),
    ("HardRainSunset", carla.WeatherParameters.HardRainSunset),
    ("SoftRainSunset", carla.WeatherParameters.SoftRainSunset),
    ("ClearNight", carla.WeatherParameters.ClearNight),
    ("CloudyNight", carla.WeatherParameters.CloudyNight),
    ("WetNight", carla.WeatherParameters.WetNight),
    ("WetCloudyNight", carla.WeatherParameters.WetCloudyNight),
    ("SoftRainNight", carla.WeatherParameters.SoftRainNight),
    ("MidRainyNight", carla.WeatherParameters.MidRainyNight),
    ("HardRainNight", carla.WeatherParameters.HardRainNight),
    ("DustStorm", carla.WeatherParameters.DustStorm)
    ("FoggyNoon", get_foggy(carla.WeatherParameters.ClearNoon)),
    ("FoggySunset", get_foggy(carla.WeatherParameters.ClearSunset)),
    ("FoggyNight", get_foggy(carla.WeatherParameters.ClearNight)),
]
