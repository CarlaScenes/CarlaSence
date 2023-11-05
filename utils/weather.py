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

def testNoonWeather(w, angle):
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
    weather.sun_altitude_angle = angle


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
    ("TestNoon1", testNoonWeather(carla.WeatherParameters.ClearNoon, 0)),
    ("TestNoon2", testNoonWeather(carla.WeatherParameters.ClearNoon, 5)),
    ("TestNoon3", testNoonWeather(carla.WeatherParameters.ClearNoon, 10)),
    ("TestNoon4", testNoonWeather(carla.WeatherParameters.ClearNoon, 15)),
    ("TestNoon5", testNoonWeather(carla.WeatherParameters.ClearNoon, 20)),
    ("TestNoon6", testNoonWeather(carla.WeatherParameters.ClearNoon, 25)),
    ("TestNoon7", testNoonWeather(carla.WeatherParameters.ClearNoon, 30)),
    ("TestNoon8", testNoonWeather(carla.WeatherParameters.ClearNoon, 35)),
    ("TestNoon9", testNoonWeather(carla.WeatherParameters.ClearNoon, 40)),
    ("TestNoon10", testNoonWeather(carla.WeatherParameters.ClearNoon, 45)),
    ("TestNoon11", testNoonWeather(carla.WeatherParameters.ClearNoon, 50)),
    ("TestNoon12", testNoonWeather(carla.WeatherParameters.ClearNoon, 55)),
    ("TestNoon13", testNoonWeather(carla.WeatherParameters.ClearNoon, 60)),
    ("TestNoon14", testNoonWeather(carla.WeatherParameters.ClearNoon, 65)),
    ("TestNoon15", testNoonWeather(carla.WeatherParameters.ClearNoon, 70)),
    ("TestNoon16", testNoonWeather(carla.WeatherParameters.ClearNoon, 75)),
    ("TestNoon17", testNoonWeather(carla.WeatherParameters.ClearNoon, 80)),
    ("TestNoon18", testNoonWeather(carla.WeatherParameters.ClearNoon, 85)),
]
