
mpd = 111194.92664455874
# 31194.926644558742
# 80000
# 11  小数点 准确

dpm = 8.993216059187304e-06
dp2m = 1.7986432118374608e-05
dp3m = 2.6979648177561914e-05

# def lat2x(lat):
#     return lat*mpd
# def lon2y(lon):
#     return lon*mpd

def x2lat(x):
    return x/mpd
def y2lon(lon):
    return lon/mpd
