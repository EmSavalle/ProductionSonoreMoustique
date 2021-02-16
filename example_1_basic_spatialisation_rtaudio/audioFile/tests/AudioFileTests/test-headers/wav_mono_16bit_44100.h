#include <vector>

namespace wav_mono_16bit_44100 { 

int numSamplesPerChannel = 352800;
int bitDepth = 16;
uint32_t sampleRate = 44100;
int numChannels = 1;

std::vector<double> testBuffer = {-3.0517578125e-05, -0.0001220703125, -0.00125122070312, -0.00265502929688, -0.00308227539062, 0.00384521484375, 0.0227661132812, 0.0328063964844, 0.02392578125, -0.00692749023438, -0.0446472167969, -0.0336303710938, 0.0135803222656, 0.0308532714844, 0.0175476074219, 0.00823974609375, 0.00466918945312, -0.00555419921875, -0.0089111328125, -0.0144958496094, -0.0140991210938, -0.0464477539062, -0.0876159667969, -0.0701599121094, -0.0661926269531, -0.0385131835938, 0.0790100097656, -0.0153503417969, 0.08154296875, 0.0448608398438, 0.139038085938, 0.130859375, 0.138977050781, 0.170257568359, 0.177337646484, 0.200958251953, 0.1513671875, 0.172027587891, 0.0646362304688, 0.0185241699219, -0.0003662109375, -0.040283203125, 0.0504150390625, 0.133178710938, -0.0211181640625, 0.0137939453125, -0.0610046386719, 0.0365600585938, 0.149353027344, -0.0780029296875, -0.0301818847656, -0.136657714844, 0.0835266113281, 0.147277832031, 0.0752868652344, 0.043701171875, 0.0494995117188, 0.259674072266, 0.239288330078, 0.329772949219, 0.241302490234, 0.261627197266, 0.145355224609, 0.160583496094, 0.232482910156, 0.0930786132812, 0.117095947266, 0.0181274414062, 0.0835571289062, 0.208099365234, 0.236755371094, 0.148284912109, 0.197235107422, 0.167083740234, 0.1513671875, 0.0177001953125, -0.0956726074219, 0.00750732421875, -0.122131347656, 0.0116577148438, 0.134368896484, 0.111206054688, 0.019775390625, 0.019287109375, 0.140533447266, 0.315887451172, 0.263671875, 0.111663818359, 0.125061035156, 0.0837097167969, 0.0767517089844, -0.158081054688, -0.163513183594, -0.114227294922, -0.08740234375, 0.0341186523438, 0.100738525391, 0.0437927246094, 0.131469726562, 0.170318603516, 0.101196289062, 0.285430908203, 0.0733642578125, 0.171081542969, 0.169342041016, 0.201751708984, 0.209381103516, -0.0686950683594, -0.00189208984375, -0.0485534667969, -0.00241088867188, -0.113494873047, -0.0881958007812, -0.159271240234, -0.113952636719, -0.0447998046875, -0.0769348144531, 0.0389709472656, -0.0861206054688, -0.127380371094, -0.132049560547, -0.0716857910156, -0.06298828125, 0.0721740722656, 0.0432434082031, 0.007568359375, -0.112457275391, -0.235748291016, -0.246368408203, -0.339935302734, -0.268218994141, -0.319763183594, -0.125457763672, -0.166229248047, -0.169342041016, -0.165557861328, -0.165771484375, -0.108154296875, -0.197784423828, -0.0548706054688, -0.187103271484, -0.0615234375, -0.0774841308594, -0.140899658203, -0.0173645019531, -0.0400695800781, 0.0636901855469, 0.0671997070312, 0.0647583007812, 0.0325622558594, 0.0327758789062, -0.0206604003906, -0.0535888671875, -0.097900390625, -0.162231445312, -0.149749755859, -0.140808105469, -0.146606445312, -0.172393798828, -0.201354980469, -0.15380859375, -0.0779113769531, -0.145324707031, -0.114990234375, -0.110107421875, -0.0872497558594, -0.0793762207031, -0.0997619628906, -0.0444030761719, -0.0523986816406, -0.05029296875, -0.0876159667969, -0.118804931641, -0.111633300781, -0.0910339355469, -0.0748596191406, -0.0581970214844, -0.127624511719, -0.0899047851562, -0.0523986816406, -0.0505676269531, -0.0670471191406, -0.115203857422, -0.0403442382812, -0.0544128417969, 0.00579833984375, 0.0446166992188, 0.110961914062, 0.163970947266, 0.249969482422, 0.233154296875, 0.126220703125, 0.254669189453, 0.203918457031, 0.184631347656, 0.142150878906, 0.0533752441406, 0.0257873535156, 0.0112609863281, -0.00982666015625, -0.0409851074219, -0.0390930175781, -0.0410766601562, 0.0111083984375, -0.0426330566406, 0.0182800292969, -0.0126342773438, -0.0324401855469, 0.0874938964844, 0.0893859863281, 0.0784912109375, 0.0954895019531, 0.125396728516, 0.0786743164062, 0.109405517578, 0.113433837891, 0.100341796875, 0.0984802246094, 0.13330078125, 0.127777099609, 0.0899658203125, 0.0910339355469, -0.0078125, 0.000946044921875, -0.0272521972656, -0.0807189941406, -0.0822448730469, -0.0395812988281, 0.0112609863281, -0.0389404296875, -0.069580078125, -0.0356140136719, 0.00152587890625, 0.0395202636719, 0.0851440429688, 0.0232849121094, 0.0352478027344, 0.0550842285156, 0.0643615722656, 0.107238769531, 0.0672302246094, 0.121002197266, 0.124298095703, 0.124237060547, 0.168914794922, 0.125244140625, 0.145721435547, 0.1474609375, 0.110626220703, 0.08642578125, 0.0535278320312, 0.0909118652344, 0.128784179688, 0.0944519042969, 0.0710754394531, 0.0735778808594, 0.0362854003906, 0.0314025878906, 0.0253601074219, 0.0295715332031, -0.00909423828125, -0.0329895019531, -0.0288696289062, -0.0804138183594, -0.0247192382812, -0.0498046875, -0.0691223144531, -0.0651245117188, -0.054931640625, -0.0852966308594, -0.101531982422, -0.0321044921875, -0.0686340332031, -0.0404357910156, -0.111938476562, -0.0735473632812, -0.0289306640625, -0.00732421875, -0.00460815429688, -0.0372619628906, 0.0257263183594, -0.076171875, -0.0304260253906, -0.0396118164062, 0.0057373046875, -0.00625610351562, -0.0441284179688, 0.00308227539062, -0.120483398438, -0.0907897949219, -0.0749816894531, -0.0639343261719, -0.104675292969, -0.133026123047, -0.163208007812, -0.13623046875, -0.151519775391, -0.208465576172, -0.132965087891, -0.143890380859, -0.152862548828, -0.167785644531, -0.0839233398438, -0.130737304688, -0.163513183594, -0.125366210938, -0.189086914062, -0.191101074219, -0.175323486328, -0.10791015625, -0.107330322266, -0.127532958984, -0.0863647460938, -0.115692138672, -0.129821777344, -0.105773925781, -0.160461425781, -0.0808410644531, -0.179809570312, -0.216979980469, -0.139892578125, -0.142639160156, -0.102600097656, -0.204376220703, -0.106140136719, -0.153381347656, -0.106536865234, -0.0618591308594, -0.0208435058594, 0.0464782714844, 0.00955200195312, 0.153717041016, 0.0352783203125, 0.136566162109, 0.116912841797, 0.128326416016, 0.154541015625, 0.0167846679688, 0.0965576171875, 0.0578308105469, 0.154266357422, 0.115325927734, 0.0791625976562, 0.126068115234, 0.0601196289062, 0.0514831542969, 0.0687866210938, 0.0473937988281, 0.0599060058594, -0.0187072753906, -0.0220642089844, 0.0264892578125, -0.0203552246094, 0.0415649414062, -0.00173950195312, 0.0784912109375, 0.0131225585938, 0.0772705078125, 0.131774902344, 0.009033203125, 0.0645446777344, -0.0211486816406, 0.0419311523438, 0.0037841796875, -0.0091552734375, 0.0363159179688, 0.0618591308594, 0.0712585449219, 0.0101623535156, 0.0268859863281, 0.0422668457031, 0.0192260742188, -0.0226135253906, 0.0460205078125, 0.0313415527344, 0.0335388183594, 0.0142517089844, 0.0131225585938, 0.0102844238281, 0.00469970703125, -0.0155639648438, 0.0622863769531, 0.0859680175781, 0.0921020507812, 0.201751708984, 0.00747680664062, 0.0858764648438, 0.0157775878906, 0.0203857421875, 0.106353759766, -0.0195617675781, 0.0973815917969, 0.0996704101562, 0.0573425292969, 0.000701904296875, 0.0412902832031, 0.0293579101562, 0.126434326172, 0.0507202148438, 0.0795593261719, 0.164245605469, 0.0859680175781, 0.180694580078, 0.133850097656, 0.130493164062, 0.0213012695312, 0.134429931641, 0.0462341308594, -0.00442504882812, 0.0191040039062, -0.0386047363281, 0.0408020019531, -0.0935363769531, -0.0704040527344, -0.0925903320312, -0.01220703125, -0.0637512207031, -0.0347900390625, 0.0331115722656, 0.0126342773438, 0.0217895507812, -0.0354919433594, 0.0665588378906, 0.00479125976562, 0.0277709960938, 0.0141296386719, -0.0735168457031, -0.0988464355469, -0.0360107421875, -0.0831298828125, -0.0945434570312, -0.128967285156, -0.159851074219, -0.0594177246094, -0.185394287109, -0.124206542969, -0.198303222656, -0.125274658203, -0.129455566406, -0.221313476562, -0.0959777832031, -0.159637451172, -0.0964660644531, -0.132995605469, -0.119171142578, -0.0887756347656, -0.0859985351562, -0.108276367188, -0.0869750976562, -0.0390625, -0.108489990234, -0.06884765625, -0.0766906738281, -0.101013183594, -0.116058349609, -0.0977478027344, -0.0970458984375, -0.16552734375, -0.170654296875, -0.123992919922, -0.179107666016, -0.132415771484, -0.129211425781, -0.126708984375, -0.0523376464844, -0.127746582031, -0.05712890625, -0.0557556152344, -0.0438537597656, -0.05029296875, -0.04345703125, -0.0101623535156, -0.05126953125, -0.0245361328125, -0.0565795898438, -0.0192260742188, -0.0189208984375, 0.00103759765625, 0.00106811523438, -0.010498046875, 0.0216674804688, 0.0352172851562, 0.0218200683594, -0.000579833984375, -0.0510559082031, -0.0981140136719, -0.046630859375, -0.0908508300781, -0.0478820800781, -0.0527648925781, -0.102416992188, -0.0680541992188, -0.0557250976562, -0.0196533203125, 0.0538330078125, 0.0516052246094, -0.0245971679688, -0.00387573242188, -0.0279846191406, 0.0260009765625};

}; // end namespace
