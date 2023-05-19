/*************************************************/
/********* DON'T MODIFY THIS FILE ****************/
/*************************************************/
#include "airports.h"

std::array<row, 303> airports = 
{{
{"Albany_NY", 42.710356, -75.053609, 131.0},{"Edison_NJ", 40.544595, -75.568613, 159.0},{"Dayton_OH", 39.858702, -85.511527, 133.0},{"Boise_ID", 43.592251, -117.51392, 143.0},{"Lumberton_NC", 34.667629, -80.236843, 105.0},{"Albuquerque_NM", 35.108486, -107.847304, 175.0},{"Newark_DE", 39.662265, -76.926527, 120.0},
{"West_Lebanon_NH", 43.623536, -73.560395, 153.0},{"West_Wendover_NV", 40.738399, -115.293498, 106.0},{"Salina_KS", 38.877342, -98.853199, 177.0},{"Glen_Allen_VA", 37.66976, -78.695914, 128.0},{"Beaver_UT", 38.249149, -113.887024, 109.0},{"Pleasant_Prairie_WI", 42.518715, -89.184928, 144.0},{"Independence_MO", 39.040814, -95.603765, 107.0},{"Redondo_Beach_CA", 33.894227, -119.601907, 114.0},{"Yuma_AZ", 32.726686, -115.853593, 116.0},{"Milford_CT", 41.245823, -74.243559, 130.0},{"Liverpool_NY", 43.102424, -77.421946, 138.0},
{"Columbia_MO", 38.957778, -93.487261, 109.0},{"Harrisburg_PA", 40.277134, -78.057755, 141.0},{"Turkey_Lake_FL", 28.514873, -82.734689, 133.0},{"Lake_City_FL", 30.181405, -83.914105, 86.0},{"Fremont_CA", 37.394181, -123.384358, 151.0},{"Bozeman_MT", 45.70007, -112.29779, 105.0},{"Peru_IL", 41.348503, -90.360615, 158.0},{"Pendleton_OR", 45.64655, -119.91648, 82.0},{"Ann_Arbor_MI", 42.241125, -85.001022, 103.0},{"Needles_CA", 34.850835, -115.858829, 102.0},{"Lebec_CA", 34.98737, -120.180772, 180.0},
{"Atlanta_GA", 33.79382, -85.63163, 145.0},{"Winnemucca_NV", 40.958869, -118.981001, 114.0},{"Queens_NY", 40.66179, -75.02732, 149.0},{"Country_Club_Hills_IL", 41.585206, -88.955614, 88.0},{"Flagstaff_AZ", 35.174151, -112.897694, 144.0},{"Norfolk_VA", 36.860525, -77.441967, 128.0},{"Uvalde_TX", 29.112378, -100.98658, 140.0},{"Tannersville_PA", 41.045431, -76.546737, 174.0},{"Centralia_WA", 46.729872, -124.211892, 159.0},{"Southampton_NY", 40.891909, -73.661495, 104.0},{"Seaside_CA", 36.61697, -123.078473, 110.0},
{"Dublin_CA", 37.703163, -123.159804, 175.0},{"Lexington_KY", 38.017955, -85.655164, 122.0},{"Napa_CA", 38.235578, -123.498386, 155.0},{"Augusta_ME", 44.347885, -71.020542, 129.0},{"Nephi_UT", 39.678111, -113.075503, 141.0},{"Green_River_UT", 38.993577, -111.375013, 146.0},{"Plattsburgh_NY", 44.704537, -74.726329, 169.0},{"Hooksett_NH", 43.109066, -72.712268, 121.0},{"Cisco_TX", 32.374263, -100.241697, 181.0},{"Cadillac_MI", 44.28254, -86.63756, 111.0},{"Cranbury_NJ", 40.32244, -75.7214, 103.0},
{"Charlotte_NC", 35.34075, -82.00029, 164.0},{"Indio_CA", 33.741291, -117.449529, 167.0},{"Alexandria_LA", 31.312424, -93.680936, 160.0},{"Maumee_OH", 41.57833, -84.899093, 134.0},{"Ellensburg_WA", 46.976918, -121.77612, 139.0},{"Savannah_GA", 32.135885, -82.447353, 100.0},{"Holbrook_AZ", 34.922962, -111.380058, 132.0},{"Fresno_CA", 36.835455, -121.14508, 113.0},{"Newburgh_NY", 41.499616, -75.305824, 194.0},{"Temecula_CA", 33.52421, -118.387068, 98.0},{"South_Burlington_VT", 44.46286, -74.413808, 117.0},
{"Folsom_CA", 38.642291, -122.42263, 148.0},{"Gardnerville_NV", 38.696385, -120.783025, 100.0},{"London_KY", 37.14916, -85.34835, 190.0},{"Casa_Grande_AZ", 32.878773, -112.916194, 158.0},{"San_Marcos_TX", 29.827707, -99.214185, 126.0},{"Corsicana_TX", 32.068583, -97.682748, 125.0},{"El_Centro_CA", 32.760837, -116.766986, 128.0},{"Onalaska_WI", 43.879042, -92.422928, 130.0},{"Darien_CT", 41.080103, -74.69585, 150.0},{"Sandy_OR", 45.402786, -123.528871, 119.0},{"Superior_MT", 47.192149, -116.123401, 125.0},
{"Manteca_CA", 37.782622, -122.463183, 128.0},{"Ocala_FL", 29.140981, -83.428438, 127.0},{"Santa_Rosa_NM", 34.947013, -105.882497, 164.0},{"Santee_SC", 33.485858, -81.710263, 188.0},{"South_Salt_Lake_City_UT", 40.720352, -113.123212, 167.0},{"Sparks_NV", 39.541124, -120.676836, 188.0},{"Allentown_PA", 40.588118, -76.794589, 183.0},{"Knoxville_TN", 35.901319, -85.384134, 126.0},{"Moab_UT", 38.573122, -110.786868, 121.0},{"Denver_CO", 39.77512, -106.029148, 160.0},{"Brandon_FL", 27.940665, -83.558025, 146.0},
{"Rapid_City_SD", 44.105601, -104.447069, 128.0},{"West_Yellowstone_MT", 44.656089, -112.333522, 135.0},{"Burlington_WA", 48.509743, -123.573181, 121.0},{"Cheyenne_WY", 41.161085, -106.039455, 179.0},{"Dedham_MA", 42.236461, -72.412825, 146.0},{"West_Springfield_MA", 42.130914, -73.855935, 139.0},{"Port_St._Lucie_FL", 27.31293, -81.641243, 129.0},{"Somerset_PA", 40.017517, -80.31162, 133.0},{"San_Rafael_CA", 37.963357, -123.750199, 89.0},{"St._Joseph_MI", 42.056357, -87.690852, 124.0},{"San_Mateo_CA", 37.5447, -123.52461, 118.0},
{"Vienna_VA", 38.931919, -78.474064, 84.0},{"Brentwood_TN", 35.9696, -88.038659, 183.0},{"Ukiah_CA", 39.1481, -124.443104, 153.0},{"Aurora_IL", 41.760671, -89.543684, 105.0},{"San_Diego_CA", 32.902166, -118.428199, 102.0},{"Hawthorne_CA", 33.921063, -119.564574, 158.0},{"Grove_City_OH", 39.877253, -84.297948, 155.0},{"Gallup_NM", 35.505278, -110.062594, 161.0},{"Butte_MT", 45.981226, -113.741661, 84.0},{"Grants_Pass_OR", 42.460931, -124.558624, 118.0},{"Queensbury_NY", 43.328388, -74.914492, 118.0},{"Colorado_Springs_CO", 38.837573, -106.059389, 158.0},
{"Highland_Park_IL", 42.17434, -89.051126, 138.0},{"Hays_KS", 38.900543, -100.553642, 156.0},{"St._Charles_MO", 38.78216, -91.7674, 115.0},{"Paramus_NJ", 40.957892, -75.308476, 114.0},{"Lone_Tree_CO", 39.563776, -106.110151, 188.0},{"Cleveland_OH", 41.519427, -82.727646, 146.0},{"Bellmead_TX", 31.582287, -98.343652, 132.0},{"Seabrook_NH", 42.895248, -72.103799, 108.0},{"Missoula_MT", 46.914375, -115.266424, 114.0},{"Watertown_NY", 43.979585, -77.188614, 166.0},{"Atascadero_CA", 35.486585, -121.900878, 94.0},{"Murdo_SD", 43.886915, -101.951387, 121.0},
{"Burbank_CA", 34.174754, -119.535303, 179.0},{"Sunnyvale_CA", 37.405893, -123.222445, 150.0},{"Laurel_MD", 39.095382, -78.092819, 115.0},{"Oakdale_MN", 44.964892, -94.195749, 130.0},{"Buffalo_NY", 42.968675, -79.93018, 146.0},{"Culver_City_CA", 33.986765, -119.624662, 120.0},{"Fountain_Valley_CA", 33.70275, -119.168797, 125.0},{"Macon_GA", 32.833485, -84.860313, 160.0},{"Baxter_MN", 46.378836, -95.490878, 142.0},{"Madison_WI", 43.12669, -90.541329, 151.0},{"Angola_IN", 41.699048, -86.234826, 129.0},{"Effingham_IL", 39.137114, -89.797968, 131.0},
{"Quartzsite_AZ", 33.660784, -115.476301, 123.0},{"Gilroy_CA", 37.02445, -122.79985, 155.0},{"Kennewick_WA", 46.198035, -120.397187, 157.0},{"Hamilton_Township_NJ", 40.195539, -75.875875, 110.0},{"Duluth_MN", 46.784467, -93.33682, 184.0},{"Terre_Haute_IN", 39.443345, -88.566237, 146.0},{"Egg_Harbor_Township_NJ", 39.393663, -75.797119, 79.0},{"Las_Vegas_NV", 36.165906, -116.373155, 84.0},{"Mammoth_Lakes_CA", 37.644519, -120.199999, 97.0},{"Strasburg_VA", 39.00496, -79.572348, 176.0},{"Wickenburg_AZ", 33.970281, -113.966003, 164.0},{"Limon_CO", 39.268975, -104.943126, 126.0},
{"East_Greenwich_RI", 41.660517, -72.731742, 107.0},{"Riviera_Beach_FL", 26.77825, -81.344086, 113.0},{"Erie_PA", 42.049602, -81.320845, 144.0},{"Kingman_AZ", 35.191331, -115.300092, 98.0},{"Okeechobee_FL", 27.60089, -82.05736, 135.0},{"Big_Timber_MT", 45.83626, -111.17791, 166.0},{"Tucumcari_NM", 35.15396, -104.9571, 147.0},{"Baton_Rouge_LA", 30.423892, -92.389137, 173.0},{"The_Dalles_OR", 45.611941, -122.442749, 178.0},{"Greenwich_CT", 41.041538, -74.906161, 138.0},{"Dallas_TX", 32.832466, -98.072138, 101.0},{"Perry_OK", 36.289315, -98.560435, 144.0},
{"Syosset_NY", 40.7999, -74.74974, 106.0},{"Cranberry_PA", 40.683508, -81.342827, 148.0},{"Greenville_SC", 34.729509, -83.600853, 96.0},{"Tonopah_NV", 38.069801, -118.466743, 119.0},{"Mountville_SC", 34.39359, -83.263298, 132.0},{"Pearl_MS", 32.274159, -91.385548, 141.0},{"Louisville_KY", 38.211962, -86.90769, 177.0},{"Buellton_CA", 34.614555, -121.422932, 155.0},{"Sheboygan_WI", 43.749753, -88.981471, 116.0},{"Bethesda_MD", 39.023876, -78.378852, 106.0},{"Victoria_TX", 28.766853, -98.213488, 165.0},{"Grand_Rapids_MI", 42.914231, -86.767557, 125.0},{"Tifton_GA", 31.448847, -84.76671, 185.0},
{"Richfield_UT", 38.78799, -113.319673, 176.0},{"Columbus_TX", 29.690066, -97.772227, 142.0},{"Indianapolis_IN", 39.702238, -87.31409, 91.0},{"Triadelphia_WV", 40.06076, -81.837242, 115.0},{"Normal_IL", 40.508562, -90.219238, 162.0},{"Burlingame_CA", 37.593182, -123.601983, 130.0},{"Mountain_View_CA", 37.415328, -123.311075, 133.0},{"South_Hill_VA", 36.748516, -79.338017, 149.0},{"Chicago_IL", 41.890872, -88.888714, 144.0},{"Brooklyn_NY", 40.68331, -75.241008, 115.0},{"Buttonwillow_CA", 35.400105, -120.632296, 166.0},{"Beatty_NV", 36.913695, -117.988963, 127.0},
{"Asheville_NC", 35.531428, -83.838995, 163.0},{"Corning_CA", 39.92646, -123.4329, 134.0},{"Shreveport_LA", 32.478594, -94.98887, 95.0},{"Farmington_NM", 36.766315, -109.378766, 143.0},{"Billings_MT", 45.734046, -109.839432, 119.0},{"Matthews_NC", 35.140024, -81.954276, 110.0},{"Twin_Falls_ID", 42.597887, -115.689749, 146.0},{"Vacaville_CA", 38.366645, -123.192636, 123.0},{"St._Augustine_FL", 29.924286, -82.650518, 137.0},{"Lake_Charles_LA", 30.199071, -94.483282, 134.0},{"Tinton_Falls_NJ", 40.226408, -75.328072, 113.0},{"Stanfield_AZ", 32.949077, -113.226433, 92.0},
{"Grand_Junction_CO", 39.090758, -109.838825, 107.0},{"Coeur_d'Alene_ID", 47.708479, -118.028783, 113.0},{"Lindale_TX", 32.470885, -96.684973, 135.0},{"Orlando_FL", 28.617982, -82.622495, 124.0},{"Binghamton_NY", 42.145542, -77.136581, 157.0},{"Hagerstown_MD", 39.605859, -78.967824, 121.0},{"DeFuniak_Springs_FL", 30.720702, -87.351177, 123.0},{"Slidell_LA", 30.266552, -90.994656, 124.0},{"Kingsland_GA", 30.790734, -82.898125, 130.0},{"Catoosa_OK", 36.167631, -97.000544, 132.0},{"Port_Huron_MI", 42.998817, -83.663435, 86.0},{"Marathon_FL", 24.72611, -82.282412, 154.0},
{"Goodland_KS", 39.326258, -102.959607, 140.0},{"Cherry_Valley_IL", 42.243893, -90.213395, 101.0},{"Truckee_CA", 39.327438, -121.44191, 158.0},{"Monterey_CA", 36.612153, -123.132495, 165.0},{"Blue_Ash_OH", 39.224642, -85.618007, 127.0},{"Rocky_Mount_NC", 35.972904, -79.081345, 180.0},{"Inyokern_CA", 35.646451, -119.047144, 178.0},{"Sagamore_Beach_MA", 41.781195, -71.774789, 114.0},{"West_Hartford_CT", 41.722672, -73.994217, 106.0},{"Hinckley_MN", 46.009797, -94.16587, 169.0},{"Bowling_Green_KY", 36.955196, -87.673354, 145.0},{"Oxnard_CA", 34.238115, -120.412584, 104.0},
{"Auburn_AL", 32.627837, -86.679605, 111.0},{"Costa_Mesa_CA", 33.673925, -119.116912, 119.0},{"Roseville_CA", 38.771208, -122.500649, 138.0},{"East_Brunswick_NJ", 40.415938, -75.679213, 153.0},{"Bellevue_WA", 47.62957, -123.382573, 145.0},{"St._George_UT", 37.126463, -114.836237, 183.0},{"Buckeye_AZ", 33.443011, -113.791376, 154.0},{"San_Juan_Capistrano_CA", 33.498538, -118.89759, 135.0},{"Oklahoma_City_OK", 35.461664, -98.88594, 87.0},{"Lima_OH", 40.726668, -85.306432, 159.0},{"Weatherford_OK", 35.53859, -99.89462, 116.0},{"Ritzville_WA", 47.116294, -119.602828, 118.0},
{"Trinidad_CO", 37.134167, -105.753852, 121.0},{"Denton_TX", 33.231373, -98.400912, 154.0},{"Sweetwater_TX", 32.450591, -101.626955, 145.0},{"Champaign_IL", 40.146204, -89.494328, 144.0},{"Gillette_WY", 44.292984, -106.760825, 135.0},{"Barstow_CA", 34.849124, -118.319959, 127.0},{"Mobile_AL", 30.671556, -89.353144, 98.0},{"Glenwood_Springs_CO", 39.552676, -108.574671, 125.0},{"Miner_MO", 36.893583, -90.768486, 153.0},{"Eureka_CA", 40.778885, -125.422883, 135.0},{"Plantation_FL", 26.108605, -81.486944, 113.0},{"Idaho_Falls_ID", 43.485152, -113.28655, 142.0},
{"Utica_NY", 43.113878, -76.441357, 133.0},{"Fort_Myers_FL", 26.485574, -83.021649, 106.0},{"Yucca_AZ", 34.879736, -115.366062, 131.0},{"Albert_Lea_MN", 43.68606, -94.592221, 92.0},{"Sheridan_WY", 44.804582, -108.190845, 95.0},{"Sulphur_Springs_TX", 33.137098, -96.837729, 151.0},{"Villa_Park_IL", 41.907415, -89.207523, 129.0},{"Mayer_AZ", 34.32753, -113.35296, 142.0},{"Gila_Bend_AZ", 32.943675, -113.968581, 131.0},{"Mishawaka_IN", 41.717337, -87.42313, 109.0},{"Tempe_AZ", 33.421676, -113.131831, 187.0},{"Silverthorne_CO", 39.631467, -107.305318, 163.0},
{"Huntsville_TX", 30.716158, -96.800444, 154.0},{"Price_UT", 39.600831, -112.066166, 163.0},{"Lone_Pine_CA", 36.60059, -119.296416, 105.0},{"Amarillo_TX", 35.189016, -103.165967, 98.0},{"Woodburn_OR", 45.15313, -124.115754, 139.0},{"Primm_NV", 35.610678, -116.622514, 115.0},{"Lincoln_City_OR", 44.957751, -125.245466, 136.0},{"Blanding_UT", 37.625618, -110.708342, 148.0},{"Brattleboro_VT", 42.838443, -73.800298, 107.0},{"Springfield_OR", 44.082607, -124.271958, 92.0},{"Cabazon_CA", 33.931316, -118.054582, 169.0},{"Pocatello_ID", 42.899615, -113.669748, 96.0},
{"Mt._Shasta_CA", 41.310222, -123.55181, 103.0},{"Decatur_GA", 33.793198, -85.519894, 128.0},{"Bend_OR", 44.03563, -122.542973, 186.0},{"Coalinga_CA", 36.254143, -121.47242, 159.0},{"Wytheville_VA", 36.945693, -82.289151, 142.0},{"Chattanooga_TN", 35.038644, -86.43043, 113.0},{"Port_Orange_FL", 29.108571, -82.269103, 165.0},{"Wichita_KS", 37.60878, -98.56764, 154.0},{"Macedonia_OH", 41.313663, -82.751518, 159.0},{"Tremonton_UT", 41.70995, -113.433076, 99.0},{"Plymouth_NC", 35.850587, -77.990616, 107.0},{"Petaluma_CA", 38.242676, -123.859523, 134.0},
{"Lafayette_IN", 40.41621, -88.048589, 126.0},{"Detroit_OR", 44.73704, -123.386499, 99.0},{"Palo_Alto_CA", 37.394011, -123.384847, 124.0},{"Mojave_CA", 35.068595, -119.409076, 169.0},{"Eau_Claire_WI", 44.77083, -92.67161, 142.0},{"Mitchell_SD", 43.701129, -99.279, 125.0},{"Lee_MA", 42.295745, -74.473726, 151.0},{"Houston_TX", 29.980687, -96.656047, 124.0},{"East_Liberty_OH", 40.303817, -84.785029, 145.0},{"Tallahassee_FL", 30.510908, -85.482341, 182.0},{"Lovelock_NV", 40.179476, -119.706635, 168.0},{"Ardmore_OK", 34.179106, -98.400132, 143.0},
{"Baker_City_OR", 44.782882, -119.046806, 163.0},{"Woodbridge_VA", 38.64082, -78.53083, 97.0},{"Rocklin_CA", 38.80086, -122.445029, 125.0},{"Elko_NV", 40.836301, -117.025359, 108.0},{"Reno_NV", 39.489732, -121.028679, 142.0},{"Lusk_WY", 42.75625, -105.68717, 136.0},{"Shamrock_TX", 35.226765, -101.48286, 173.0},{"Tooele_UT", 40.684466, -113.503508, 126.0},{"Salisbury_MD", 38.4016, -76.79939, 108.0},{"Council_Bluffs_IA", 41.220921, -97.070079, 165.0},{"Topeka_KS", 39.04438, -96.994767, 122.0},{"Rancho_Cucamonga_CA", 34.113584, -118.763927, 108.0},
{"Worthington_MN", 43.63385, -96.830147, 108.0},{"Mauston_WI", 43.795551, -91.293858, 138.0},{"Warsaw_NC", 34.994625, -79.37017, 135.0}
}};

/*************************************************/
/********* DON'T THIS FILE ***********************/
/*************************************************/
