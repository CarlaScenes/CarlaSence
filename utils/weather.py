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
    ("carla.WeatherParameters.ClearNoon", carla.WeatherParameters.ClearNoon),
    ("carla.WeatherParameters.MidRainyNoon", carla.WeatherParameters.MidRainyNoon),
    ("carla.WeatherParameters.ClearSunset", carla.WeatherParameters.ClearSunset),
    ("carla.WeatherParameters.MidRainSunset",
     carla.WeatherParameters.MidRainSunset),
    ("carla.WeatherParameters.ClearNight", carla.WeatherParameters.ClearNight),
    ("carla.WeatherParameters.MidRainyNight",
     carla.WeatherParameters.MidRainyNight),
    ("FoggyNoon", get_foggy(carla.WeatherParameters.ClearNoon)),
    ("FoggySunset", get_foggy(carla.WeatherParameters.ClearSunset)),
    ("FoggyNight", get_foggy(carla.WeatherParameters.ClearNight)),
]
