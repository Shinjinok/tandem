// this is an auto-generated file from the IGRF tables. Do not edit
// To re-generate run generate/generate.py

#include "AP_Declination.h"

const float AP_Declination::SAMPLING_RES = 10;
const float AP_Declination::SAMPLING_MIN_LAT = -90;
const float AP_Declination::SAMPLING_MAX_LAT = 90;
const float AP_Declination::SAMPLING_MIN_LON = -180;
const float AP_Declination::SAMPLING_MAX_LON = 180;

const float AP_Declination::declination_table[19][37] = {
    {148.69761f,138.69761f,128.69761f,118.69761f,108.69761f,98.69761f,88.69761f,78.69761f,68.69761f,58.69761f,48.69761f,38.69761f,28.69761f,18.69762f,8.69762f,-1.30238f,-11.30238f,-21.30238f,-31.30238f,-41.30238f,-51.30238f,-61.30238f,-71.30238f,-81.30238f,-91.30238f,-101.30238f,-111.30238f,-121.30238f,-131.30238f,-141.30238f,-151.30238f,-161.30238f,-171.30238f,178.69762f,168.69762f,158.69762f,148.69762f},
    {128.94897f,116.76913f,105.67868f,95.53037f,86.14242f,77.33693f,68.95887f,60.88360f,53.01792f,45.29703f,37.67824f,30.13241f,22.63434f,15.15427f,7.65257f,0.07962f,-7.61922f,-15.49516f,-23.58758f,-31.91892f,-40.49507f,-49.31107f,-58.36060f,-67.64675f,-77.19175f,-87.04449f,-97.28490f,-108.02444f,-119.40021f,-131.55717f,-144.60993f,-158.57795f,-173.30675f,171.57451f,156.59245f,142.26654f,128.94898f},
    {85.84983f,77.86674f,71.41911f,65.88958f,60.87391f,56.06040f,51.19464f,46.08566f,40.62541f,34.80211f,28.69654f,22.45697f,16.25163f,10.20429f,4.33277f,-1.47753f,-7.44138f,-13.79058f,-20.68196f,-28.14192f,-36.06877f,-44.28287f,-52.59545f,-60.86678f,-69.03901f,-77.14829f,-85.33211f,-93.85484f,-103.18780f,-114.22254f,-128.80731f,-150.82949f,175.07148f,137.78538f,112.24670f,96.50833f,85.84983f},
    {48.36464f,46.92931f,45.32796f,43.77215f,42.30938f,40.81602f,38.99843f,36.47427f,32.92196f,28.21415f,22.48505f,16.13188f,9.73548f,3.87119f,-1.15119f,-5.47488f,-9.65651f,-14.39119f,-20.17189f,-27.07282f,-34.75143f,-42.64839f,-50.23639f,-57.15663f,-63.21910f,-68.33458f,-72.41907f,-75.25804f,-76.25303f,-73.68371f,-61.52538f,-19.19465f,28.93504f,44.30881f,48.57111f,49.20204f,48.36464f},
    {31.53591f,31.70110f,31.36033f,30.84826f,30.41740f,30.18801f,29.98071f,29.24535f,27.24713f,23.40449f,17.58459f,10.28432f,2.61324f,-4.10075f,-8.99321f,-12.12560f,-14.37513f,-16.96134f,-20.98285f,-26.89095f,-34.11467f,-41.48727f,-48.01539f,-53.12365f,-56.47783f,-57.79090f,-56.64764f,-52.28187f,-43.58389f,-29.99471f,-13.30334f,2.47967f,14.54407f,22.66040f,27.66680f,30.39349f,31.53591f},
    {22.75929f,23.28778f,23.31689f,23.03727f,22.65340f,22.42715f,22.46484f,22.41917f,21.39316f,18.26620f,12.36103f,4.08881f,-4.87185f,-12.38211f,-17.23167f,-19.66536f,-20.63683f,-21.01155f,-22.04133f,-25.40510f,-31.13118f,-37.35943f,-42.41983f,-45.46286f,-46.00424f,-43.75462f,-38.59656f,-30.61647f,-20.78459f,-11.10186f,-2.81091f,4.20460f,10.19518f,15.12619f,18.89707f,21.40949f,22.75929f},
    {17.13796f,17.65436f,17.82923f,17.72121f,17.34914f,16.86971f,16.53895f,16.35924f,15.58913f,12.78706f,6.80889f,-1.92242f,-11.14737f,-18.31589f,-22.44690f,-24.22321f,-24.54440f,-23.32152f,-20.82014f,-19.64192f,-21.94844f,-26.36091f,-30.36616f,-32.31812f,-31.48131f,-27.99265f,-22.46433f,-15.56763f,-8.66921f,-3.36126f,0.42713f,3.88155f,7.48442f,10.94407f,13.89448f,15.98811f,17.13796f},
    {13.43267f,13.72984f,13.83121f,13.80883f,13.53332f,12.95768f,12.34733f,11.91321f,11.03246f,8.16041f,2.06661f,-6.53115f,-15.00078f,-21.00237f,-23.91298f,-24.38289f,-23.12151f,-19.94479f,-14.99564f,-10.46439f,-9.07591f,-11.27253f,-15.01498f,-17.63307f,-17.72669f,-15.57904f,-11.94492f,-7.31271f,-2.89019f,-0.16068f,1.26721f,2.99592f,5.53070f,8.30122f,10.79219f,12.56424f,13.43267f},
    {11.16594f,11.17420f,11.05268f,11.01366f,10.83016f,10.28739f,9.65089f,9.13014f,8.01069f,4.83374f,-1.27984f,-9.23186f,-16.48061f,-21.11665f,-22.52269f,-21.07704f,-17.72292f,-13.25291f,-8.39083f,-4.09404f,-1.55231f,-1.87486f,-4.59894f,-7.45509f,-8.63646f,-8.08079f,-6.27170f,-3.44628f,-0.63448f,0.69183f,0.91983f,1.83959f,3.96370f,6.49529f,8.82606f,10.48601f,11.16594f},
    {9.93680f,9.77161f,9.44849f,9.39726f,9.32087f,8.86119f,8.25242f,7.59797f,6.07624f,2.49109f,-3.48144f,-10.52720f,-16.49466f,-19.74802f,-19.62688f,-16.66092f,-12.24395f,-7.75504f,-4.00456f,-0.97754f,1.30754f,1.86066f,0.22938f,-2.14752f,-3.61977f,-3.92009f,-3.31437f,-1.84169f,-0.23550f,0.22478f,-0.15913f,0.36070f,2.32394f,4.86971f,7.33323f,9.19174f,9.93680f},
    {9.16766f,9.22301f,8.96998f,9.05040f,9.17994f,8.85632f,8.14273f,7.04951f,4.85721f,0.74686f,-5.09769f,-11.24884f,-15.91183f,-17.76124f,-16.47400f,-12.93578f,-8.52466f,-4.45587f,-1.48556f,0.65504f,2.47094f,3.28762f,2.33751f,0.47598f,-0.90035f,-1.50746f,-1.60788f,-1.18479f,-0.64975f,-0.92128f,-1.74282f,-1.58396f,0.15010f,2.75445f,5.55988f,7.94494f,9.16766f},
    {8.08128f,8.94848f,9.30190f,9.80683f,10.28289f,10.15853f,9.23409f,7.43740f,4.30433f,-0.55474f,-6.49684f,-11.90459f,-15.25892f,-15.82082f,-13.84894f,-10.34859f,-6.32308f,-2.62807f,0.02035f,1.76195f,3.20429f,4.02335f,3.49629f,2.08842f,0.91631f,0.23699f,-0.27841f,-0.69925f,-1.22283f,-2.34193f,-3.71136f,-4.03497f,-2.68713f,-0.13040f,3.00221f,6.02006f,8.08128f},
    {6.28865f,8.39010f,9.89871f,11.18038f,12.10311f,12.18173f,11.06740f,8.56770f,4.39433f,-1.43294f,-7.76626f,-12.71270f,-15.03488f,-14.66135f,-12.32097f,-8.94635f,-5.20951f,-1.70287f,0.94178f,2.66658f,3.94405f,4.75690f,4.65082f,3.80530f,2.91867f,2.17198f,1.25586f,0.00097f,-1.68841f,-3.88301f,-5.99412f,-6.86672f,-5.92715f,-3.49607f,-0.18656f,3.31723f,6.28865f},
    {4.19181f,7.51880f,10.35561f,12.63336f,14.09976f,14.37492f,13.11591f,10.01912f,4.81787f,-2.13799f,-9.14859f,-14.03243f,-15.82840f,-14.94987f,-12.36184f,-8.92842f,-5.19751f,-1.62974f,1.27973f,3.35990f,4.87193f,5.97860f,6.53093f,6.48543f,6.00351f,5.07642f,3.48642f,1.09616f,-2.04223f,-5.55636f,-8.52151f,-9.86583f,-9.18150f,-6.81331f,-3.39189f,0.45296f,4.19181f},
    {2.44930f,6.67910f,10.58077f,13.82764f,15.99477f,16.63478f,15.31208f,11.56591f,5.09688f,-3.36260f,-11.40963f,-16.58147f,-18.23622f,-17.12824f,-14.31043f,-10.60907f,-6.56332f,-2.58630f,0.94938f,3.85297f,6.21157f,8.18832f,9.76938f,10.74752f,10.82421f,9.67705f,7.05532f,2.94954f,-2.20573f,-7.39132f,-11.26285f,-12.88680f,-12.18277f,-9.67581f,-6.04860f,-1.87654f,2.44930f},
    {1.23996f,6.06390f,10.65932f,14.67051f,17.60758f,18.85329f,17.63570f,13.03920f,4.54289f,-6.40446f,-16.02930f,-21.57020f,-23.02479f,-21.57781f,-18.34966f,-14.12077f,-9.40540f,-4.57538f,0.08000f,4.38118f,8.28412f,11.80076f,14.83887f,17.08578f,18.00457f,16.91952f,13.20211f,6.71258f,-1.49391f,-9.13013f,-14.13842f,-15.91086f,-14.95215f,-12.10495f,-8.12855f,-3.56708f,1.23995f},
    {-0.19763f,5.10158f,10.17946f,14.70431f,18.17900f,19.82197f,18.38042f,12.06562f,-0.18311f,-14.99150f,-25.85012f,-30.62031f,-30.82870f,-28.22309f,-23.94668f,-18.68430f,-12.86076f,-6.76373f,-0.60456f,5.45714f,11.29506f,16.77831f,21.70932f,25.74808f,28.32459f,28.52180f,24.98098f,16.33586f,3.40270f,-8.96109f,-16.46228f,-18.93755f,-17.92564f,-14.79372f,-10.45157f,-5.46722f,-0.19763f},
    {-4.95869f,0.29715f,5.10983f,8.97015f,11.07279f,10.02983f,3.61936f,-9.91862f,-26.65753f,-38.34210f,-42.90167f,-42.53669f,-39.25812f,-34.25216f,-28.18554f,-21.44863f,-14.28598f,-6.86380f,0.69398f,8.28472f,15.81220f,23.17216f,30.23342f,36.80725f,42.59003f,47.04093f,49.09660f,46.49392f,34.74459f,11.29282f,-10.25497f,-19.79571f,-21.46253f,-19.28356f,-15.22752f,-10.26709f,-4.95869f},
    {-167.92994f,-157.92994f,-147.92994f,-137.92994f,-127.92994f,-117.92994f,-107.92994f,-97.92994f,-87.92994f,-77.92994f,-67.92994f,-57.92994f,-47.92994f,-37.92994f,-27.92994f,-17.92994f,-7.92994f,2.07006f,12.07006f,22.07006f,32.07006f,42.07006f,52.07006f,62.07006f,72.07006f,82.07006f,92.07006f,102.07006f,112.07006f,122.07006f,132.07006f,142.07006f,152.07006f,162.07006f,172.07006f,-177.92994f,-167.92994f}
};

const float AP_Declination::inclination_table[19][37] = {
    {-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f,-71.99130f},
    {-78.19286f,-77.42559f,-76.50501f,-75.47258f,-74.36398f,-73.21159f,-72.04706f,-70.90264f,-69.81087f,-68.80253f,-67.90360f,-67.13235f,-66.49781f,-66.00055f,-65.63583f,-65.39826f,-65.28614f,-65.30416f,-65.46312f,-65.77726f,-66.25982f,-66.91849f,-67.75199f,-68.74833f,-69.88490f,-71.12962f,-72.44256f,-73.77717f,-75.08120f,-76.29722f,-77.36401f,-78.22001f,-78.81056f,-79.09868f,-79.07486f,-78.75915f,-78.19286f},
    {-80.75581f,-78.93373f,-77.10287f,-75.24592f,-73.33881f,-71.36672f,-69.33969f,-67.30521f,-65.35116f,-63.59354f,-62.14862f,-61.09631f,-60.44879f,-60.14166f,-60.05719f,-60.07235f,-60.10859f,-60.16019f,-60.29204f,-60.61285f,-61.23674f,-62.24694f,-63.67377f,-65.49323f,-67.64254f,-70.04134f,-72.60758f,-75.26315f,-77.93055f,-80.52222f,-82.91539f,-84.87982f,-85.93103f,-85.59287f,-84.26179f,-82.55897f,-80.75581f},
    {-77.42917f,-75.40632f,-73.46305f,-71.54198f,-69.56112f,-67.42319f,-65.05318f,-62.46126f,-59.79510f,-57.34278f,-55.46821f,-54.48140f,-54.48296f,-55.27691f,-56.43846f,-57.50838f,-58.18201f,-58.39141f,-58.28950f,-58.18795f,-58.45653f,-59.38645f,-61.08501f,-63.47193f,-66.36159f,-69.55892f,-72.91368f,-76.32082f,-79.69004f,-82.91058f,-85.76734f,-87.41518f,-86.36059f,-84.14258f,-81.81436f,-79.56382f,-77.42917f},
    {-71.57132f,-69.59858f,-67.69212f,-65.83896f,-63.98948f,-62.02187f,-59.75394f,-57.05517f,-54.00988f,-51.03013f,-48.82603f,-48.15108f,-49.34770f,-52.02590f,-55.26453f,-58.15418f,-60.14054f,-61.00633f,-60.79410f,-59.89581f,-59.07413f,-59.12802f,-60.44073f,-62.87676f,-66.01317f,-69.41135f,-72.75913f,-75.83462f,-78.38144f,-80.07841f,-80.69068f,-80.26587f,-79.09293f,-77.47048f,-75.59376f,-73.59217f,-71.57132f},
    {-64.39688f,-62.39113f,-60.38919f,-58.40045f,-56.45850f,-54.54114f,-52.48714f,-50.02973f,-47.02241f,-43.80692f,-41.42667f,-41.24864f,-43.92718f,-48.69183f,-54.01373f,-58.75437f,-62.46033f,-64.87137f,-65.59575f,-64.55571f,-62.54632f,-61.00483f,-61.02307f,-62.70229f,-65.39589f,-68.29172f,-70.81830f,-72.63613f,-73.51562f,-73.52004f,-73.01311f,-72.24482f,-71.22213f,-69.89961f,-68.26861f,-66.38970f,-64.39688f},
    {-55.03330f,-52.86020f,-50.66418f,-48.41190f,-46.16131f,-44.02436f,-41.99114f,-39.73354f,-36.79390f,-33.34156f,-30.83024f,-31.39854f,-35.89525f,-42.84574f,-50.04165f,-56.27501f,-61.39987f,-65.36559f,-67.56684f,-67.40255f,-65.17866f,-62.25826f,-60.40328f,-60.48937f,-61.99755f,-63.84135f,-65.27049f,-65.88972f,-65.50475f,-64.47761f,-63.50156f,-62.76852f,-61.95575f,-60.79105f,-59.18528f,-57.19505f,-55.03330f},
    {-42.24914f,-39.70734f,-37.26696f,-34.79237f,-32.24269f,-29.80059f,-27.59836f,-25.23497f,-22.02046f,-18.15458f,-15.64557f,-17.27466f,-23.81728f,-33.13711f,-42.44946f,-50.19511f,-56.14799f,-60.48164f,-62.92781f,-63.03901f,-60.86478f,-57.31281f,-54.14039f,-52.78754f,-53.09920f,-54.02088f,-54.79410f,-54.88214f,-53.88973f,-52.32343f,-51.24222f,-50.79055f,-50.23394f,-49.09222f,-47.27754f,-44.87944f,-42.24914f},
    {-25.33127f,-22.24275f,-19.59098f,-17.06270f,-14.40635f,-11.83113f,-9.51200f,-6.90742f,-3.35115f,0.59107f,2.59363f,-0.02013f,-7.84684f,-18.91060f,-30.19477f,-39.30172f,-45.43047f,-48.94032f,-50.34960f,-49.87131f,-47.42534f,-43.46893f,-39.69866f,-37.74333f,-37.53183f,-38.07865f,-38.72916f,-38.83816f,-37.75771f,-36.07349f,-35.23922f,-35.34725f,-35.15500f,-33.96982f,-31.77859f,-28.72099f,-25.33127f},
    {-5.24467f,-1.65627f,1.04223f,3.36830f,5.82312f,8.21859f,10.40278f,12.95422f,16.27483f,19.47808f,20.53313f,17.52733f,9.92477f,-1.10178f,-12.78858f,-22.11073f,-27.74481f,-30.09065f,-30.28390f,-29.16609f,-26.54428f,-22.40060f,-18.38152f,-16.26847f,-15.94332f,-16.39122f,-17.08995f,-17.45454f,-16.70739f,-15.39300f,-15.15307f,-16.04422f,-16.43552f,-15.41476f,-13.00600f,-9.37673f,-5.24467f},
    {14.63354f,18.26566f,20.79124f,22.76210f,24.82173f,26.91073f,28.88398f,31.08803f,33.63275f,35.67707f,35.80090f,32.87434f,26.53716f,17.48213f,7.82189f,0.10605f,-4.37281f,-5.73575f,-5.10792f,-3.65024f,-1.23229f,2.47014f,6.11733f,8.06340f,8.41024f,8.11338f,7.56325f,7.09175f,7.31886f,7.86810f,7.37562f,5.78413f,4.64459f,5.00015f,6.96986f,10.43983f,14.63353f},
    {30.98292f,34.04736f,36.26582f,37.98433f,39.77693f,41.73026f,43.68222f,45.63900f,47.48293f,48.57495f,48.02662f,45.30200f,40.43697f,34.10197f,27.67376f,22.60079f,19.68037f,19.03199f,19.95050f,21.41367f,23.33965f,26.03030f,28.69124f,30.17478f,30.50177f,30.39265f,30.14740f,29.85381f,29.76575f,29.61923f,28.58068f,26.63113f,24.88860f,24.31792f,25.24745f,27.67581f,30.98292f},
    {43.32774f,45.46848f,47.31116f,48.95263f,50.72206f,52.71614f,54.77369f,56.69575f,58.21101f,58.80693f,57.93799f,55.49094f,51.87099f,47.77570f,43.99216f,41.13433f,39.54968f,39.36154f,40.22675f,41.49006f,42.92593f,44.63908f,46.28882f,47.29570f,47.63489f,47.71684f,47.75223f,47.72175f,47.58338f,47.06921f,45.74527f,43.68775f,41.64041f,40.35994f,40.24069f,41.35214f,43.32774f},
    {53.10346f,54.36706f,55.83221f,57.46351f,59.32436f,61.38996f,63.49807f,65.41295f,66.81185f,67.24367f,66.35979f,64.28556f,61.57835f,58.86465f,56.59780f,55.01138f,54.21264f,54.23131f,54.89285f,55.84490f,56.86260f,57.91330f,58.89796f,59.63659f,60.11350f,60.47362f,60.78850f,60.97960f,60.87132f,60.20078f,58.76930f,56.74225f,54.64404f,53.01893f,52.20036f,52.27922f,53.10345f},
    {61.90994f,62.59113f,63.72281f,65.24538f,67.08381f,69.10954f,71.13858f,72.93795f,74.19279f,74.52210f,73.71918f,72.01230f,69.92626f,67.95214f,66.38550f,65.34071f,64.83436f,64.83068f,65.21810f,65.81898f,66.48539f,67.16198f,67.84471f,68.53475f,69.23882f,69.94816f,70.58468f,70.97994f,70.90797f,70.17159f,68.74684f,66.86847f,64.93030f,63.30032f,62.21297f,61.75730f,61.90994f},
    {70.57581f,70.96901f,71.80210f,73.03021f,74.56969f,76.29227f,78.02626f,79.54496f,80.54339f,80.70312f,79.93943f,78.52844f,76.88461f,75.33874f,74.08351f,73.19868f,72.69163f,72.52570f,72.63249f,72.92739f,73.34042f,73.84480f,74.45815f,75.21475f,76.12353f,77.12512f,78.06096f,78.67686f,78.70105f,78.00545f,76.71219f,75.11306f,73.51596f,72.15595f,71.17602f,70.64307f,70.57581f},
    {78.81629f,79.04888f,79.58074f,80.38163f,81.39985f,82.55681f,83.73578f,84.75483f,85.34159f,85.24161f,84.48824f,83.36449f,82.13774f,80.98018f,79.99133f,79.22173f,78.68791f,78.38311f,78.28678f,78.37466f,78.62920f,79.04674f,79.63676f,80.41039f,81.35966f,82.43079f,83.49269f,84.30787f,84.57311f,84.14411f,83.20057f,82.04885f,80.92669f,79.97993f,79.28974f,78.89660f,78.81629f},
    {85.90253f,85.97015f,86.17873f,86.51047f,86.93603f,87.40703f,87.83875f,88.08791f,88.00214f,87.58373f,86.96949f,86.28021f,85.59035f,84.94653f,84.38009f,83.91255f,83.55819f,83.32576f,83.22006f,83.24338f,83.39665f,83.67976f,84.09086f,84.62451f,85.26903f,86.00279f,86.78851f,87.56014f,88.18696f,88.43183f,88.17396f,87.64830f,87.08530f,86.59228f,86.21779f,85.98485f,85.90253f},
    {88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f,88.22541f}
};

const float AP_Declination::intensity_table[19][37] = {
    {0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f,0.54460f},
    {0.60513f,0.59870f,0.59074f,0.58150f,0.57118f,0.56001f,0.54822f,0.53607f,0.52386f,0.51190f,0.50051f,0.48998f,0.48062f,0.47267f,0.46637f,0.46193f,0.45954f,0.45938f,0.46159f,0.46626f,0.47338f,0.48283f,0.49436f,0.50758f,0.52199f,0.53701f,0.55202f,0.56637f,0.57948f,0.59085f,0.60007f,0.60691f,0.61124f,0.61309f,0.61256f,0.60983f,0.60513f},
    {0.62946f,0.61607f,0.60099f,0.58441f,0.56643f,0.54710f,0.52656f,0.50511f,0.48332f,0.46192f,0.44173f,0.42350f,0.40770f,0.39458f,0.38421f,0.37663f,0.37208f,0.37100f,0.37402f,0.38178f,0.39473f,0.41290f,0.43588f,0.46276f,0.49230f,0.52302f,0.55336f,0.58174f,0.60673f,0.62714f,0.64220f,0.65163f,0.65557f,0.65457f,0.64937f,0.64075f,0.62946f},
    {0.61817f,0.59902f,0.57889f,0.55788f,0.53572f,0.51196f,0.48615f,0.45832f,0.42923f,0.40043f,0.37385f,0.35117f,0.33322f,0.31978f,0.30989f,0.30259f,0.29761f,0.29571f,0.29842f,0.30756f,0.32462f,0.35005f,0.38308f,0.42191f,0.46420f,0.50754f,0.54955f,0.58791f,0.62041f,0.64530f,0.66159f,0.66929f,0.66921f,0.66264f,0.65105f,0.63582f,0.61817f},
    {0.58399f,0.56089f,0.53765f,0.51447f,0.49102f,0.46637f,0.43930f,0.40902f,0.37619f,0.34306f,0.31296f,0.28911f,0.27312f,0.26412f,0.25925f,0.25564f,0.25208f,0.24938f,0.24997f,0.25736f,0.27501f,0.30447f,0.34455f,0.39195f,0.44267f,0.49311f,0.54052f,0.58241f,0.61635f,0.64051f,0.65423f,0.65812f,0.65359f,0.64238f,0.62609f,0.60615f,0.58399f},
    {0.53896f,0.51414f,0.48950f,0.46541f,0.44189f,0.41825f,0.39293f,0.36452f,0.33296f,0.30028f,0.27053f,0.24845f,0.23691f,0.23451f,0.23651f,0.23850f,0.23873f,0.23744f,0.23626f,0.23929f,0.25268f,0.28075f,0.32300f,0.37471f,0.42940f,0.48162f,0.52816f,0.56692f,0.59601f,0.61462f,0.62342f,0.62358f,0.61633f,0.60307f,0.58497f,0.56309f,0.53896f},
    {0.48769f,0.46354f,0.43951f,0.41591f,0.39318f,0.37133f,0.34948f,0.32619f,0.30039f,0.27282f,0.24721f,0.22913f,0.22223f,0.22494f,0.23188f,0.23899f,0.24528f,0.25011f,0.25217f,0.25324f,0.26016f,0.28060f,0.31747f,0.36685f,0.42026f,0.46978f,0.51130f,0.54275f,0.56267f,0.57233f,0.57509f,0.57248f,0.56433f,0.55098f,0.53306f,0.51140f,0.48769f},
    {0.43200f,0.41070f,0.38966f,0.36904f,0.34940f,0.33115f,0.31429f,0.29781f,0.27991f,0.26008f,0.24106f,0.22782f,0.22394f,0.22859f,0.23779f,0.24873f,0.26117f,0.27375f,0.28254f,0.28593f,0.28844f,0.29847f,0.32308f,0.36210f,0.40745f,0.44990f,0.48433f,0.50765f,0.51788f,0.51831f,0.51535f,0.51083f,0.50262f,0.48994f,0.47326f,0.45334f,0.43200f},
    {0.37884f,0.36269f,0.34716f,0.33242f,0.31890f,0.30691f,0.29660f,0.28738f,0.27757f,0.26597f,0.25371f,0.24387f,0.23964f,0.24244f,0.25136f,0.26421f,0.27946f,0.29518f,0.30752f,0.31365f,0.31502f,0.31799f,0.33084f,0.35643f,0.38917f,0.42117f,0.44732f,0.46348f,0.46689f,0.46142f,0.45466f,0.44840f,0.43964f,0.42735f,0.41238f,0.39571f,0.37884f},
    {0.34109f,0.33182f,0.32331f,0.31594f,0.31035f,0.30643f,0.30374f,0.30178f,0.29921f,0.29424f,0.28628f,0.27690f,0.26908f,0.26633f,0.27092f,0.28146f,0.29468f,0.30805f,0.31935f,0.32662f,0.32964f,0.33204f,0.33997f,0.35599f,0.37700f,0.39833f,0.41618f,0.42664f,0.42709f,0.42023f,0.41143f,0.40221f,0.39103f,0.37791f,0.36439f,0.35184f,0.34109f},
    {0.32814f,0.32507f,0.32303f,0.32261f,0.32490f,0.32947f,0.33494f,0.34007f,0.34325f,0.34214f,0.33531f,0.32395f,0.31144f,0.30222f,0.29995f,0.30449f,0.31281f,0.32258f,0.33240f,0.34074f,0.34704f,0.35337f,0.36247f,0.37454f,0.38823f,0.40215f,0.41413f,0.42111f,0.42116f,0.41485f,0.40385f,0.38947f,0.37320f,0.35713f,0.34342f,0.33369f,0.32814f},
    {0.33976f,0.33989f,0.34245f,0.34778f,0.35682f,0.36891f,0.38185f,0.39331f,0.40094f,0.40198f,0.39471f,0.38068f,0.36427f,0.35055f,0.34312f,0.34224f,0.34613f,0.35355f,0.36326f,0.37307f,0.38214f,0.39195f,0.40311f,0.41441f,0.42544f,0.43669f,0.44699f,0.45365f,0.45459f,0.44849f,0.43453f,0.41418f,0.39144f,0.37044f,0.35399f,0.34380f,0.33976f},
    {0.37235f,0.37273f,0.37794f,0.38769f,0.40180f,0.41892f,0.43653f,0.45192f,0.46239f,0.46491f,0.45764f,0.44227f,0.42373f,0.40754f,0.39706f,0.39269f,0.39344f,0.39879f,0.40765f,0.41763f,0.42745f,0.43793f,0.44954f,0.46153f,0.47375f,0.48647f,0.49847f,0.50709f,0.50961f,0.50355f,0.48754f,0.46343f,0.43615f,0.41083f,0.39087f,0.37800f,0.37235f},
    {0.42247f,0.42195f,0.42802f,0.44001f,0.45649f,0.47519f,0.49349f,0.50903f,0.51943f,0.52209f,0.51541f,0.50071f,0.48226f,0.46507f,0.45238f,0.44506f,0.44276f,0.44510f,0.45117f,0.45920f,0.46796f,0.47773f,0.48930f,0.50293f,0.51833f,0.53452f,0.54942f,0.56021f,0.56398f,0.55837f,0.54263f,0.51885f,0.49159f,0.46568f,0.44455f,0.42997f,0.42247f},
    {0.48322f,0.48204f,0.48709f,0.49773f,0.51225f,0.52823f,0.54326f,0.55533f,0.56273f,0.56380f,0.55766f,0.54519f,0.52913f,0.51298f,0.49944f,0.48986f,0.48458f,0.48349f,0.48606f,0.49132f,0.49853f,0.50774f,0.51962f,0.53461f,0.55217f,0.57054f,0.58702f,0.59869f,0.60306f,0.59862f,0.58556f,0.56599f,0.54349f,0.52179f,0.50364f,0.49054f,0.48322f},
    {0.53930f,0.53779f,0.54027f,0.54630f,0.55479f,0.56418f,0.57284f,0.57936f,0.58261f,0.58179f,0.57657f,0.56743f,0.55568f,0.54310f,0.53141f,0.52187f,0.51527f,0.51197f,0.51198f,0.51505f,0.52093f,0.52960f,0.54122f,0.55571f,0.57222f,0.58905f,0.60388f,0.61439f,0.61885f,0.61659f,0.60820f,0.59543f,0.58065f,0.56622f,0.55392f,0.54477f,0.53930f},
    {0.57279f,0.57078f,0.57043f,0.57151f,0.57359f,0.57607f,0.57830f,0.57962f,0.57952f,0.57764f,0.57388f,0.56839f,0.56161f,0.55422f,0.54698f,0.54063f,0.53583f,0.53306f,0.53263f,0.53467f,0.53921f,0.54615f,0.55528f,0.56611f,0.57786f,0.58940f,0.59945f,0.60684f,0.61077f,0.61101f,0.60791f,0.60235f,0.59544f,0.58828f,0.58177f,0.57650f,0.57279f},
    {0.57909f,0.57732f,0.57582f,0.57458f,0.57352f,0.57253f,0.57150f,0.57030f,0.56884f,0.56704f,0.56490f,0.56248f,0.55987f,0.55726f,0.55486f,0.55288f,0.55155f,0.55106f,0.55155f,0.55309f,0.55568f,0.55922f,0.56354f,0.56838f,0.57340f,0.57824f,0.58254f,0.58598f,0.58836f,0.58960f,0.58974f,0.58893f,0.58741f,0.58544f,0.58327f,0.58110f,0.57909f},
    {0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f,0.56852f}
};

