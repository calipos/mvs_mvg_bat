#include <iostream>
#include <set>
#include <vector>
#include <numeric>
#include <fstream>
#include <string> 
#include <sstream> 
#include "glog/logging.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "types.h"
#include "SelectViews.h" 
#include "omp.h"

#include "cereal/cereal.hpp"
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/utility.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/complex.hpp"
#include "cereal/types/base_class.hpp"
#include "cereal/types/array.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/tuple.hpp"
#define IGL_RAY_TRI_EPSILON 0.000000001
#define IGL_RAY_TRI_CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define IGL_RAY_TRI_DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define IGL_RAY_TRI_SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2]; 

template<class Dtype = float>
int intersect_triangle1(const Dtype* orig, const Dtype* dir,
	const Dtype* vert0, const Dtype* vert1, const Dtype* vert2,
	Dtype* t, Dtype* u, Dtype* v)
{
	Dtype edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	Dtype det, inv_det;

	/* find vectors for two edges sharing vert0 */
	IGL_RAY_TRI_SUB(edge1, vert1, vert0);
	IGL_RAY_TRI_SUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	IGL_RAY_TRI_CROSS(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = IGL_RAY_TRI_DOT(edge1, pvec);

	if (det > IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		if (*u < 0.0 || *u > det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v < 0.0 || *u + *v > det)
			return 0;

	}
	else if (det < -IGL_RAY_TRI_EPSILON)
	{
		/* calculate distance from vert0 to ray origin */
		IGL_RAY_TRI_SUB(tvec, orig, vert0);

		/* calculate U parameter and test bounds */
		*u = IGL_RAY_TRI_DOT(tvec, pvec);
		/*      printf("*u=%f\n",(float)*u); */
		/*      printf("det=%f\n",det); */
		if (*u > 0.0 || *u < det)
			return 0;

		/* prepare to test V parameter */
		IGL_RAY_TRI_CROSS(qvec, tvec, edge1);

		/* calculate V parameter and test bounds */
		*v = IGL_RAY_TRI_DOT(dir, qvec);
		if (*v > 0.0 || *u + *v < det)
			return 0;
	}
	else return 0;  /* ray is parallel to the plane of the triangle */


	inv_det = 1.0 / det;

	/* calculate t, ray intersects triangle */
	*t = IGL_RAY_TRI_DOT(edge2, qvec) * inv_det;
	(*u) *= inv_det;
	(*v) *= inv_det;

	return 1;
}
std::vector<cv::Point3i>mediaPipeFaces = { {17, 18, 313}, {17, 18, 83}, {38, 81, 82}, {13, 38, 82}, {8, 9, 285}, {8, 9, 55}, {248, 281, 456}, 
	{248, 419, 456}, {2, 97, 167}, {2, 164, 167}, {271, 303, 304}, {271, 302, 303}, {69, 104, 105}, {67, 69, 104}, {253, 449, 450}, 
	{252, 253, 450}, {41, 42, 81}, {41, 42, 74}, {16, 17, 315}, {15, 16, 315}, {40, 73, 74}, {41, 73, 74}, {339, 373, 390}, {254, 339, 373},
	{258, 384, 385}, {258, 385, 386}, {63, 104, 105}, {52, 63, 105}, {48, 115, 219}, {48, 219, 235}, {272, 304, 408}, {271, 272, 304}, 
	{174, 217, 236}, {198, 217, 236}, {22, 26, 231}, {22, 230, 231}, {17, 84, 85}, {84, 85, 180}, {364, 365, 367}, {365, 367, 397}, 
	{218, 220, 237}, {79, 218, 237}, {112, 243, 244}, {112, 133, 243}, {427, 432, 436}, {425, 427, 436}, {270, 409, 410}, {287, 409, 410},
	{266, 329, 330}, {266, 330, 425}, {135, 136, 138}, {135, 136, 150}, {88, 95, 96}, {394, 430, 434}, {394, 430, 431}, {9, 107, 108}, 
	{69, 107, 108}, {50, 117, 118}, {117, 118, 229}, {344, 438, 439}, {392, 438, 439}, {257, 442, 443}, {282, 442, 443}, {260, 466, 467},
	{260, 388, 466}, {48, 115, 131}, {276, 342, 353}, {276, 342, 445}, {121, 128, 232}, {47, 121, 128}, {18, 200, 421}, {200, 421, 428}, 
	{35, 113, 124}, {46, 113, 124}, {132, 137, 177}, {93, 132, 137}, {23, 229, 230}, {22, 23, 230}, {343, 357, 412}, {277, 343, 357},
	{283, 443, 444}, {259, 443, 444}, {62, 78, 96}, {62, 78, 191}, {26, 112, 155}, {26, 154, 155}, {24, 110, 228}, {25, 110, 228},
	{427, 432, 434}, {422, 432, 434}, {364, 379, 394}, {364, 365, 379}, {308, 324, 325}, {292, 308, 325}, {422, 424, 430}, {424, 430, 431},
	{36, 101, 205}, {36, 205, 206}, {360, 363, 420}, {363, 420, 456}, {21, 71, 162}, {250, 328, 462}, {326, 328, 462}, {351, 412, 419}, 
	{6, 351, 419}, {345, 352, 366}, {345, 366, 447}, {8, 168, 193}, {8, 55, 193}, {281, 363, 456}, {42, 80, 183}, {80, 183, 191}, 
	{307, 321, 375}, {306, 307, 375}, {2, 94, 370}, {2, 94, 141}, {284, 298, 333}, {284, 332, 333}, {17, 313, 314}, {40, 185, 186}, 
	{40, 74, 185}, {1, 19, 44}, {1, 4, 44}, {273, 287, 291}, {287, 291, 409}, {280, 352, 411}, {352, 376, 411}, {20, 238, 242}, {20, 79, 238}, 
	{59, 75, 166}, {59, 166, 219}, {272, 310, 311}, {272, 310, 407}, {420, 437, 456}, {420, 429, 437}, {259, 260, 387}, {260, 387, 388}, 
	{25, 130, 226}, {130, 226, 247}, {4, 45, 51}, {45, 51, 134}, {152, 175, 377}, {175, 377, 396}, {202, 210, 214}, {202, 212, 214}, 
	{353, 372, 383}, {276, 353, 383}, {254, 339, 448}, {255, 339, 448}, {291, 408, 409}, {291, 306, 408}, {264, 356, 454}, {264, 447, 454}, 
	{274, 440, 457}, {438, 440, 457}, {355, 429, 437}, {355, 358, 429}, {88, 89, 96}, {89, 90, 96}, {187, 205, 207}, {187, 207, 214},
	{278, 344, 360}, {278, 279, 360}, {292, 306, 307}, {342, 446, 467}, {359, 446, 467}, {39, 92, 165}, {39, 165, 167}, {9, 151, 337}, 
	{151, 337, 338}, {122, 168, 193}, {41, 72, 73}, {318, 402, 403}, {318, 319, 403}, {263, 466, 467}, {263, 359, 467}, {283, 444, 445},
	{276, 283, 445}, {42, 74, 184}, {18, 83, 201}, {83, 182, 201}, {52, 65, 222}, {65, 221, 222}, {0, 11, 37}, {11, 37, 72}, {272, 407, 408},
	{89, 179, 180}, {85, 179, 180}, {36, 100, 101}, {36, 100, 142}, {28, 157, 158}, {344, 438, 440}, {294, 331, 358}, {279, 331, 358}, 
	{136, 138, 172}, {314, 315, 404}, {314, 404, 405}, {8, 168, 417}, {8, 285, 417}, {34, 127, 234}, {34, 127, 139}, {367, 416, 433}, 
	{364, 367, 416}, {90, 180, 181}, {84, 180, 181}, {322, 426, 436}, {322, 391, 426}, {197, 248, 419}, {6, 197, 419}, {309, 457, 459}, 
	{309, 438, 457}, {138, 172, 215}, {138, 213, 215}, {141, 241, 242}, {238, 241, 242}, {33, 130, 247}, {33, 246, 247}, {53, 224, 225},
	{46, 53, 225}, {126, 129, 209}, {126, 209, 217}, {29, 223, 224}, {53, 223, 224}, {2, 97, 141}, {62, 183, 191}, {202, 204, 210}, 
	{43, 202, 204}, {78, 95, 96}, {161, 246, 247}, {123, 147, 177}, {147, 177, 215}, {55, 65, 107}, {55, 65, 221}, {116, 123, 137}, 
	{116, 137, 227}, {135, 150, 169}, {256, 381, 382}, {256, 341, 382}, {171, 199, 208}, {171, 175, 199}, {293, 298, 333}, {293, 298, 301}, 
	{106, 194, 204}, {43, 106, 204}, {149, 170, 176}, {149, 150, 170}, {315, 403, 404}, {315, 316, 403}, {57, 61, 185}, {43, 57, 61}, 
	{50, 187, 205}, {4, 44, 45}, {279, 358, 429}, {277, 329, 355}, {277, 355, 437}, {309, 392, 438}, {42, 80, 81}, {174, 196, 236}, 
	{128, 232, 233}, {12, 268, 302}, {268, 271, 302}, {313, 314, 405}, {64, 98, 240}, {64, 235, 240}, {273, 291, 375}, {291, 306, 375}, 
	{46, 113, 225}, {113, 225, 247}, {65, 66, 107}, {30, 225, 247}, {280, 346, 347}, {346, 347, 449}, {19, 44, 125}, {44, 125, 237},
	{378, 379, 395}, {250, 458, 459}, {250, 309, 459}, {50, 101, 205}, {35, 111, 226}, {35, 111, 143}, {76, 77, 146}, {62, 76, 77}, 
	{122, 193, 245}, {198, 209, 217}, {131, 198, 209}, {32, 194, 201}, {182, 194, 201}, {306, 407, 408}, {9, 285, 336}, {285, 295, 336}, 
	{2, 164, 393}, {2, 326, 393}, {133, 190, 243}, {189, 190, 243}, {9, 336, 337}, {34, 227, 234}, {93, 227, 234}, {276, 283, 293}, 
	{276, 293, 300}, {28, 56, 221}, {28, 221, 222}, {57, 185, 186}, {114, 174, 217}, {131, 134, 220}, {131, 134, 198}, {278, 344, 439}, 
	{343, 399, 412}, {343, 399, 437}, {28, 158, 159}, {128, 233, 245}, {128, 188, 245}, {341, 463, 464}, {413, 463, 464}, {257, 259, 443},
	{364, 394, 434}, {37, 164, 167}, {10, 109, 151}, {3, 196, 236}, {256, 341, 452}, {341, 452, 453}, {2, 326, 370}, {307, 320, 321}, 
	{307, 320, 325}, {0, 37, 164}, {250, 290, 328}, {101, 119, 120}, {101, 118, 119}, {399, 419, 456}, {399, 437, 456}, {12, 13, 268}, 
	{12, 13, 38}, {115, 218, 219}, {115, 218, 220}, {16, 17, 85}, {9, 55, 107}, {22, 145, 153}, {22, 153, 154}, {344, 360, 440},
	{267, 269, 393}, {269, 391, 393}, {364, 416, 434}, {38, 41, 72}, {38, 41, 81}, {296, 299, 336}, {295, 296, 336}, {63, 68, 104},
	{63, 68, 71}, {316, 317, 402}, {252, 253, 374}, {253, 373, 374}, {67, 103, 104}, {250, 309, 392}, {23, 144, 145}, {22, 23, 145}, 
	{46, 63, 70}, {63, 70, 71}, {292, 308, 415}, {292, 407, 415}, {21, 54, 68}, {6, 122, 168}, {6, 168, 351}, {12, 38, 72}, {194, 204, 211}, 
	{17, 83, 84}, {83, 84, 181}, {282, 283, 293}, {282, 293, 334}, {120, 231, 232}, {120, 230, 231}, {411, 427, 434}, {320, 321, 405}, 
	{320, 404, 405}, {266, 423, 426}, {391, 423, 426}, {258, 286, 441}, {286, 414, 441}, {341, 453, 464}, {200, 201, 208}, {199, 200, 208}, 
	{250, 458, 462}, {112, 232, 233}, {26, 112, 232}, {258, 286, 384}, {335, 418, 424}, {418, 424, 431}, {27, 159, 160}, {286, 384, 398}, 
	{1, 19, 274}, {1, 4, 274}, {18, 200, 201}, {111, 116, 123}, {111, 117, 123}, {169, 210, 211}, {204, 210, 211}, {280, 330, 347}, 
	{330, 347, 348}, {422, 430, 434}, {135, 192, 214}, {135, 169, 214}, {280, 346, 352}, {265, 340, 446}, {265, 342, 446}, {189, 243, 244},
	{168, 351, 417}, {351, 417, 465}, {110, 144, 163}, {7, 110, 163}, {347, 348, 450}, {348, 349, 450}, {369, 377, 396}, {193, 244, 245},
	{233, 244, 245}, {83, 181, 182}, {349, 450, 451}, {349, 451, 452}, {98, 129, 203}, {98, 165, 203}, {376, 401, 435}, {376, 433, 435}, 
	{50, 101, 118}, {345, 372, 447}, {357, 412, 465}, {238, 239, 241}, {77, 90, 96}, {77, 90, 91}, {369, 395, 400}, {369, 395, 431},
	{253, 254, 449}, {15, 315, 316}, {411, 416, 433}, {299, 336, 337}, {46, 124, 156}, {46, 70, 156}, {37, 39, 167}, {290, 328, 460}, 
	{290, 305, 460}, {410, 432, 436}, {287, 410, 432}, {277, 350, 357}, {35, 113, 226}, {55, 189, 193}, {55, 189, 221}, {17, 314, 315}, 
	{67, 108, 109}, {67, 69, 108}, {4, 275, 281}, {4, 274, 275}, {34, 143, 156}, {124, 143, 156}, {425, 426, 436}, {52, 53, 223}, 
	{354, 370, 461}, {370, 461, 462}, {23, 24, 144}, {29, 30, 224}, {112, 233, 244}, {330, 348, 349}, {340, 345, 352}, {340, 345, 372}, 
	{6, 122, 196}, {288, 361, 401}, {361, 366, 401}, {24, 228, 229}, {252, 256, 381}, {252, 380, 381}, {187, 192, 213}, {187, 192, 214},
	{58, 132, 177}, {58, 177, 215}, {316, 402, 403}, {279, 360, 429}, {140, 170, 176}, {140, 148, 176}, {22, 26, 154}, {52, 53, 63}, 
	{79, 166, 218}, {20, 79, 166}, {71, 139, 162}, {169, 210, 214}, {19, 354, 370}, {32, 140, 171}, {32, 140, 211}, {30, 160, 161}, 
	{29, 30, 160}, {57, 186, 212}, {57, 202, 212}, {7, 25, 33}, {251, 298, 301}, {251, 301, 389}, {138, 192, 213}, {32, 194, 211}, 
	{40, 92, 186}, {52, 66, 105}, {274, 354, 457}, {354, 457, 461}, {379, 394, 395}, {273, 321, 375}, {319, 320, 325}, {14, 316, 317}, 
	{19, 274, 354}, {14, 15, 316}, {14, 15, 86}, {260, 445, 467}, {319, 320, 404}, {329, 349, 350}, {349, 350, 452}, {265, 342, 353}, 
	{74, 184, 185}, {43, 61, 146}, {61, 76, 146}, {113, 226, 247}, {321, 405, 406}, {122, 188, 245}, {54, 103, 104}, {19, 94, 370}, 
	{19, 125, 141}, {19, 94, 141}, {280, 411, 425}, {115, 131, 220}, {85, 86, 179}, {15, 85, 86}, {265, 353, 372}, {265, 340, 372}, 
	{44, 220, 237}, {118, 119, 230}, {347, 449, 450}, {3, 195, 197}, {3, 196, 197}, {458, 459, 461}, {458, 461, 462}, {169, 170, 211}, 
	{150, 169, 170}, {32, 201, 208}, {304, 408, 409}, {270, 304, 409}, {282, 295, 296}, {282, 295, 442}, {34, 139, 156}, {42, 183, 184},
	{326, 391, 393}, {326, 327, 391}, {123, 147, 187}, {100, 126, 142}, {47, 100, 126}, {350, 357, 452}, {21, 68, 71}, {15, 16, 85}, 
	{297, 299, 333}, {297, 332, 333}, {260, 444, 445}, {54, 68, 104}, {264, 372, 383}, {285, 413, 417}, {20, 60, 99}, {20, 60, 166}, 
	{39, 72, 73}, {39, 40, 73}, {123, 137, 177}, {64, 102, 129}, {49, 102, 129}, {134, 198, 236}, {175, 199, 396}, {269, 302, 303}, 
	{293, 333, 334}, {86, 87, 178}, {14, 86, 87}, {98, 99, 240}, {60, 99, 240}, {49, 64, 102}, {256, 451, 452}, {297, 337, 338}, 
	{297, 299, 337}, {264, 368, 383}, {97, 141, 242}, {114, 174, 188}, {174, 188, 196}, {119, 120, 230}, {30, 224, 225}, {48, 49, 131},
	{266, 425, 426}, {120, 121, 232}, {100, 120, 121}, {28, 56, 157}, {323, 361, 366}, {93, 137, 227}, {273, 422, 424}, {249, 339, 390}, 
	{6, 196, 197}, {26, 231, 232}, {273, 287, 422}, {277, 329, 350}, {276, 300, 383}, {195, 248, 281}, {300, 368, 383}, {357, 453, 465}, 
	{357, 452, 453}, {310, 407, 415}, {266, 329, 371}, {266, 371, 423}, {360, 420, 429}, {292, 306, 407}, {112, 133, 155}, {264, 356, 368}, 
	{253, 254, 373}, {35, 124, 143}, {289, 392, 439}, {289, 439, 455}, {13, 268, 312}, {318, 319, 325}, {275, 281, 363}, {0, 164, 267},
	{164, 267, 393}, {39, 40, 92}, {189, 193, 244}, {34, 143, 227}, {293, 300, 301}, {62, 76, 183}, {10, 151, 338}, {186, 212, 216}, 
	{92, 186, 216}, {108, 109, 151}, {189, 190, 221}, {273, 335, 424}, {257, 258, 442}, {258, 441, 442}, {295, 441, 442}, {47, 114, 128}, 
	{47, 114, 217}, {133, 173, 190}, {43, 91, 106}, {289, 290, 305}, {289, 305, 455}, {362, 398, 414}, {362, 414, 463}, {43, 57, 202}, 
	{252, 256, 451}, {126, 129, 142}, {329, 355, 371}, {355, 358, 371}, {262, 369, 431}, {77, 91, 146}, {4, 5, 51}, {254, 448, 449}, 
	{27, 29, 160}, {411, 416, 434}, {23, 24, 229}, {66, 69, 105}, {88, 89, 179}, {27, 29, 223}, {326, 370, 462}, {378, 395, 400}, 
	{90, 91, 181}, {91, 181, 182}, {252, 374, 380}, {24, 110, 144}, {18, 313, 421}, {61, 184, 185}, {89, 90, 180}, {116, 143, 227}, 
	{252, 450, 451}, {285, 413, 441}, {117, 228, 229}, {31, 117, 228}, {140, 170, 211}, {46, 53, 63}, {92, 206, 216}, {205, 206, 216},
	{3, 51, 195}, {88, 178, 179}, {86, 178, 179}, {75, 235, 240}, {59, 75, 235}, {399, 412, 419}, {287, 422, 432}, {327, 358, 423}, 
	{327, 391, 423}, {0, 11, 267}, {352, 376, 401}, {32, 171, 208}, {30, 161, 247}, {166, 218, 219}, {278, 279, 294}, {282, 296, 334}, 
	{296, 299, 334}, {44, 45, 220}, {45, 134, 220}, {3, 51, 236}, {51, 134, 236}, {20, 99, 242}, {257, 259, 387}, {273, 321, 335}, 
	{394, 395, 431}, {285, 295, 441}, {457, 459, 461}, {255, 359, 446}, {376, 411, 433}, {249, 255, 263}, {58, 172, 215}, {52, 65, 66},
	{36, 203, 206}, {262, 369, 396}, {207, 212, 214}, {280, 330, 425}, {259, 260, 444}, {122, 188, 196}, {406, 418, 421}, {335, 406, 418}, 
	{286, 398, 414}, {358, 371, 423}, {271, 272, 311}, {5, 195, 281}, {5, 51, 195}, {50, 123, 187}, {125, 141, 241}, {48, 49, 64}, 
	{262, 418, 421}, {262, 418, 431}, {413, 414, 463}, {294, 455, 460}, {278, 294, 455}, {118, 229, 230}, {251, 284, 298}, {322, 410, 436}, 
	{270, 322, 410}, {195, 197, 248}, {31, 111, 226}, {43, 91, 146}, {106, 182, 194}, {91, 106, 182}, {165, 203, 206}, {7, 25, 110}, 
	{261, 346, 448}, {255, 261, 448}, {147, 213, 215}, {48, 64, 235}, {278, 439, 455}, {267, 269, 302}, {129, 142, 203}, {36, 142, 203}, 
	{268, 271, 311}, {305, 455, 460}, {66, 69, 107}, {27, 28, 222}, {27, 222, 223}, {97, 98, 99}, {9, 108, 151}, {264, 372, 447}, 
	{323, 366, 447}, {70, 139, 156}, {56, 190, 221}, {27, 28, 159}, {288, 397, 435}, {367, 397, 435}, {92, 165, 206}, {25, 33, 130},
	{100, 101, 120}, {282, 283, 443}, {411, 425, 427}, {279, 294, 331}, {319, 403, 404}, {47, 126, 217}, {147, 187, 213}, {257, 258, 386},
	{257, 386, 387}, {205, 207, 216}, {269, 322, 391}, {148, 171, 175}, {11, 267, 302}, {11, 12, 302}, {352, 366, 401}, {59, 219, 235},
	{125, 237, 241}, {237, 239, 241}, {369, 377, 400}, {346, 448, 449}, {326, 327, 328}, {261, 340, 346}, {340, 346, 352}, {25, 31, 228},
	{269, 270, 322}, {50, 117, 123}, {61, 76, 184}, {76, 183, 184}, {261, 340, 446}, {255, 261, 446}, {289, 290, 392}, {250, 290, 392}, 
	{97, 98, 165}, {52, 222, 223}, {148, 152, 175}, {60, 75, 166}, {249, 255, 339}, {321, 335, 406}, {140, 148, 171}, {294, 327, 460}, 
	{255, 263, 359}, {56, 157, 173}, {56, 173, 190}, {31, 111, 117}, {356, 368, 389}, {301, 368, 389}, {351, 412, 465}, {268, 311, 312}, 
	{288, 401, 435}, {318, 324, 325}, {97, 99, 242}, {49, 131, 209}, {413, 414, 441}, {114, 128, 188}, {135, 138, 192}, {262, 396, 428}, 
{262, 421, 428}, {25, 31, 226}, {11, 12, 72}, {79, 237, 239}, {79, 238, 239}, {111, 116, 143}, {199, 396, 428}, {127, 139, 162}, {417, 464, 465},
 {413, 417, 464}, {342, 445, 467}, {453, 464, 465}, {207, 212, 216}, {292, 307, 325}, {313, 405, 406}, {329, 330, 349}, {97, 165, 167}, 
{323, 447, 454}, {60, 75, 240}, {300, 301, 368}, {360, 363, 440}, {299, 333, 334}, {70, 71, 139}, {294, 327, 358}, {62, 77, 96}, {277, 343, 437},
 {47, 100, 121}, {199, 200, 428}, {341, 362, 463}, {64, 98, 129}, {269, 270, 303}, {49, 129, 209}, {327, 328, 460}, {313, 406, 421}, 
{341, 362, 382}, {4, 5, 281}, {367, 433, 435}, {275, 363, 440}, {274, 275, 440}, {270, 303, 304}, {37, 39, 72} };
cv::Point_<dType> project(const Camera& camera, cv::Point3_<dType>& pt, const Eigen::Quaternion<dType>& q, const Eigen::Matrix<dType, 3, 1>& t)
{ 
	Eigen::Matrix<dType, 4, 4>Rt = Eigen::Matrix<dType, 4, 4>::Identity();
	Rt.topLeftCorner<3, 3>() = q.matrix();
	Rt.topRightCorner<3, 1>() = t; 
	Eigen::Matrix<dType, 4, 1> objPt(pt.x, pt.y, pt.z,1);
	auto cameraPt = Rt* objPt;
	dType u = cameraPt[0] / cameraPt[2];
	dType v = cameraPt[1] / cameraPt[2]; 
	return cv::Point_<dType>(u * camera.focalLength + camera.cx, v * camera.focalLength + camera.cy);
}
void testQt()
{
	std::string colmapDir = "D:/repo/mvs_mvg_bat/viewerout/colmap";
	if (colmapDir[colmapDir.length() - 1] == '\\' || colmapDir[colmapDir.length() - 1] == '/')
	{

	}
	else
	{
		colmapDir += "/";
	}
	std::string cameraTXT = colmapDir + "cameras.txt";
	std::string imagesTXT = colmapDir + "images.txt";
	std::string points3DTXT = colmapDir + "points3D.txt";
	std::vector<Camera>cameras = Camera::readCameras(cameraTXT);
	std::map<int, ImageData>imgs = ImageData::readImageData(imagesTXT);
	std::map<int,Point3dData>objPts = Point3dData::readPoint3dData(points3DTXT);
	int a = 0;
	int b = 10;
	const std::vector<int>& thisObjPtsIdxA = imgs[a].thisObjPtsIdx;
	const std::vector<int>& thisObjPtsIdxB = imgs[b].thisObjPtsIdx;
	int pt_a=0;//选择这张图象的第一个  obj 点,并得到该点的3d 索引
	int objPtsIdx = thisObjPtsIdxA[pt_a];
	auto iter = std::find(thisObjPtsIdxB.begin(), thisObjPtsIdxB.end(), objPtsIdx);
	CHECK(thisObjPtsIdxB.end()!= iter);
	int pt_b = iter -thisObjPtsIdxB.begin();
	cv::Point_<dType> imgPt_a = imgs[a].thisImgPts[pt_a];
	cv::Point_<dType> imgPt_b = imgs[b].thisImgPts[pt_b];
	LOG(INFO) << "objPt = " << objPts[objPtsIdx].objPt;
	LOG(INFO) << "a = " << imgs[a].imgPath << "  " << imgPt_a << " " << project(cameras[0], objPts[objPtsIdx].objPt, imgs[a].q, imgs[a].t);
	LOG(INFO) << "b = " << imgs[b].imgPath << "  " << imgPt_b << " " << project(cameras[0], objPts[objPtsIdx].objPt, imgs[b].q, imgs[b].t); 

	return;
}
std::vector<cv::Point_<dType>> getStandardImgPts(const cv::Size& size)
{
	std::vector<cv::Point_<dType>> ret(size.width* size.height);
#pragma omp parallel for
	for (int i = 0; i < ret.size(); i++)
	{
		ret[i].x = i % size.width;
		ret[i].y = i / size.width;
	}
	return ret;
}
std::vector<cv::Point_<dType>> getFlowImgPts(const std::vector<cv::Point_<dType>>&standardImgPts,const std::string&flowPath)
{
	std::vector<cv::Point_<dType>> flowImgPts(standardImgPts.size());
	size_t size = 0;
	{
		std::ifstream in(flowPath);
		in.seekg(0, std::ios::end);
		size = in.tellg();
		in.close(); 
	}
	CHECK(standardImgPts.size() * 2 * sizeof(float) == size);
	std::vector<float> flowData(standardImgPts.size() * 2);
	{
		std::ifstream in(flowPath,std::ios::binary);
		in.read((char*)&flowData[0],size);
		in.close();
	}
#pragma omp parallel for
	for (int i = 0; i < standardImgPts.size(); i++)
	{
		flowImgPts[i].x = standardImgPts[i].x + flowData[2 * i];
		flowImgPts[i].y = standardImgPts[i].y + flowData[2 * i+1];;
	}
	return flowImgPts;
}
template<typename Dtype>
inline int writeObj(const std::string& path, const std::vector<cv::Point3_<Dtype>>& pts, const std::vector<cv::Point3i>& faces)
{
	std::fstream fout(path, std::ios::out);
	for (int i = 0; i < pts.size(); i++)
	{
		fout << "v " << pts[i].x << " " << pts[i].y << " " << pts[i].z << std::endl;
	}
	for (int i = 0; i < faces.size(); i++)
	{
		fout << "f " << faces[i].x + 1 << " " << faces[i].y + 1 << " " << faces[i].z + 1 << std::endl;
	}
	fout.close();
	return 0;
}
struct Pt3d_MeshId
{
	std::map<int, int>pt3d_MeshId;
	template <class Archive>
	void serialize(Archive& ar)
	{
		ar(cereal::make_nvp("pt3d_MeshId", pt3d_MeshId));
	}
};
void test_aff()
{
	cv::Mat mask = cv::Mat::zeros(960,540,CV_8UC1);
	cv::Mat rgb = cv::Mat::zeros(960, 540, CV_8UC3);
	std::vector<cv::Point_<int>>trianglePts{
					cv::Point_<int>(90, 90),
					cv::Point_<int>(30, 60),
					cv::Point_<int>(60, 30) };

	cv::drawContours(mask, std::vector<std::vector<cv::Point_<int>>>{ trianglePts }, 0, cv::Scalar(255), -1);
	return;
}
int main(int argc, char** argv)
{
	test_aff();
	if (argc != 2)
	{
		std::cout << "cmd colmapDir" << std::endl;
		return -1;
	}
	else
	{
		std::cout << " You called:" << std::endl;
		std::cout << "  ---" << argv[0] << std::endl;
		std::cout << "  ---" << argv[1] << std::endl;
	}
	std::string colmapDir(argv[1]);
	if (colmapDir[colmapDir.length() - 1] == '\\' || colmapDir[colmapDir.length() - 1] == '/' )
	{

	}
	else
	{
		colmapDir += "/";
	}
	std::string cameraTXT = colmapDir + "cameras.txt";
	std::string imagesTXT = colmapDir + "images.txt";
	std::string points3DTXT = colmapDir + "points3D.txt";
	std::vector<Camera>cameras = Camera::readCameras(cameraTXT);
	std::map<int, ImageData>imgs = ImageData::readImageData(imagesTXT);
	std::map<int, Point3dData>objPts = Point3dData::readPoint3dData(points3DTXT);
	std::map<int, int>objPtsToLandmarkdId;
	std::map<int, int>landmarkdIdToObjPts;

	std::string pt3dMeshIdPath = colmapDir + "../sfm/pt3dMeshId.json";
	Pt3d_MeshId pt3dMeshId;
	try
	{
		std::fstream fin(pt3dMeshIdPath, std::ios::in);
		std::stringstream ss;
		std::string aline;
		while (std::getline(fin, aline))
		{
			ss << aline;
		}
		{
			cereal::JSONInputArchive archive(ss);
			archive(cereal::make_nvp("pt3dMeshId", pt3dMeshId.pt3d_MeshId));
		}
		if (objPts.size() != pt3dMeshId.pt3d_MeshId.size())
		{
			LOG(INFO) << "objPts.size() != pt3dMeshId.pt3d_MeshId.size()";
			throw std::exception();
		}
	}
	catch (const std::exception&)
	{
		return EXIT_FAILURE;
	}
	try
	{
		std::vector<int>temp;
		temp.reserve(498);
		for (const auto&d: mediaPipeFaces)
		{
			if (temp.end() == std::find(temp.begin(), temp.end(), d.x))
			{
				temp.emplace_back(d.x);
			}
			if (temp.end() == std::find(temp.begin(), temp.end(), d.y))
			{
				temp.emplace_back(d.y);
			}
			if (temp.end() == std::find(temp.begin(), temp.end(), d.z))
			{
				temp.emplace_back(d.z);
			}
		}
		std::sort(temp.begin(), temp.end());
		CHECK(temp.size() == 468 || temp.size() == 478 );
		CHECK(temp[temp.size() - 1] == temp.size() - 1);
	}
	catch (const std::exception&)
	{
		return EXIT_FAILURE;
	}
	try
	{
		for (const auto&d: objPts)
		{
			const int& objPtId = d.first;
			CHECK(pt3dMeshId.pt3d_MeshId.count(objPtId)!=0);
			const int&landmarkId = pt3dMeshId.pt3d_MeshId[objPtId];
			objPtsToLandmarkdId[objPtId] = landmarkId;
			landmarkdIdToObjPts[landmarkId] = objPtId;
		}
		CHECK(objPts.size()== objPtsToLandmarkdId.size());
	}
	catch (const std::exception&)
	{
		return EXIT_FAILURE;
	}


	std::map<int, int>meshPtsMapToObjId;
	std::map<int, int>objIdMapToMeshPts;
	std::vector<cv::Point3_<dType>> meshPts;
	std::vector<cv::Point3i> meshFace;
	{			
		for (int f = 0; f < mediaPipeFaces.size(); f++)
		{
			const int& pa = mediaPipeFaces[f].x;
			const int& pb = mediaPipeFaces[f].y;
			const int& pc = mediaPipeFaces[f].z;
			if (landmarkdIdToObjPts.count(pa)!=0
				&& landmarkdIdToObjPts.count(pb) != 0
				&& landmarkdIdToObjPts.count(pc)!=0)
			{
				const int& objPaId = landmarkdIdToObjPts[pa];
				const int& objPbId = landmarkdIdToObjPts[pb];
				const int& objPcId = landmarkdIdToObjPts[pc];
				int meshPtsMapSize = objIdMapToMeshPts.size();
				if (objIdMapToMeshPts.count(objPaId) == 0)
				{
					objIdMapToMeshPts[objPaId] = meshPtsMapSize++;
					meshPts.emplace_back(objPts[objPaId].objPt);
				}
				if (objIdMapToMeshPts.count(objPbId) == 0)
				{
					objIdMapToMeshPts[objPbId] = meshPtsMapSize++;
					meshPts.emplace_back(objPts[objPbId].objPt);
				}
				if (objIdMapToMeshPts.count(objPcId) == 0)
				{
					objIdMapToMeshPts[objPcId] = meshPtsMapSize++;
					meshPts.emplace_back(objPts[objPcId].objPt);
				}
				meshFace.emplace_back(objIdMapToMeshPts[objPaId], objIdMapToMeshPts[objPbId], objIdMapToMeshPts[objPcId]);
			}
		}
		writeObj("mvg.obj", meshPts, meshFace);
		for (auto&d: objIdMapToMeshPts)
		{
			meshPtsMapToObjId[d.second] = d.first;
		}
	}
	std::vector<std::vector<dType>>sigmas(meshPts.size());
	for (const auto&img: imgs)
	{
		const int& imgId = img.first;
		std::string imgPath = colmapDir +"/" + img.second.imgPath;
		LOG(INFO) << "imgId=" << imgId << " : " << imgPath;
		const int& cameraId = img.second.cameraId;
		cv::Mat pic = cv::imread(imgPath);
		Eigen::Matrix<dType, 3, 1>  thisCameraT = img.second.camera_t;


#pragma omp parallel for
		for (int meshPtsIdx = 0; meshPtsIdx < meshPts.size(); meshPtsIdx++)
		{
			const int& objPtId = landmarkdIdToObjPts[meshPtsMapToObjId[meshPtsIdx]];
			const auto& objPt = objPts[objPtId].objPt;
			const auto& iter = std::find(img.second.thisObjPtsIdx.begin(), img.second.thisObjPtsIdx.end(), objPtId);
			if (img.second.thisObjPtsIdx.end() != iter)
			{
				cv::Point_<dType> imgPt = img.second.thisImgPts[iter- img.second.thisObjPtsIdx.begin()];
				cv::Point_<dType> ptReproj = cameras.at(imgs.at(img.second.imageId).cameraId).ptInView(imgs.at(img.second.imageId).worldPtInView(objPt)); 
				dType thisPtSigma = cv::norm(imgPt- ptReproj);
				sigmas[meshPtsIdx].emplace_back(thisPtSigma);
			} 
		} 
	}

	std::vector<dType>sigma(meshPts.size());
#pragma omp parallel for
	for (int i = 0; i < meshPts.size(); i++)
	{
		sigma[i] = std::accumulate(sigmas[i].begin(), sigmas[i].end(), 0.)/ sigmas[i].size();
	}
	for (const auto& img : imgs)
	{
		const int& imgId = img.first;
		std::string imgPath = colmapDir + "/" + img.second.imgPath;
		const int& cameraId = img.second.cameraId;
		const int& imgHeight = cameras[cameraId].height;
		const int& imgWidth = cameras[cameraId].width;
		cv::Mat mask = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);
		cv::Mat pixelMesh = cv::Mat::ones(imgHeight, imgWidth, CV_32SC1) * -1;
		cv::Mat pixelDist = cv::Mat::zeros(imgHeight, imgWidth, CV_32FC1);
		cv::Mat pixelSigma = cv::Mat::zeros(imgHeight, imgWidth, CV_32FC1);
		for (int f = 0; f < meshFace.size(); f++)
		{
			const int& pa = meshFace[f].x;
			const int& pb = meshFace[f].y;
			const int& pc = meshFace[f].z;
			//if (landmarkVisiables[imgId][pa] && landmarkVisiables[imgId][pb] && landmarkVisiables[imgId][pc])
			{
				const int& objPtAId = meshPtsMapToObjId[pa]; 
				const int& objPtBId = meshPtsMapToObjId[pb];
				const int& objPtCId = meshPtsMapToObjId[pc];
				const auto& iterA = std::find(img.second.thisObjPtsIdx.begin(), img.second.thisObjPtsIdx.end(), objPtAId);
				const auto& iterB = std::find(img.second.thisObjPtsIdx.begin(), img.second.thisObjPtsIdx.end(), objPtBId);
				const auto& iterC = std::find(img.second.thisObjPtsIdx.begin(), img.second.thisObjPtsIdx.end(), objPtCId);

				cv::Point3_<dType> cameraPtA = imgs.at(img.second.imageId).worldPtInView(meshPts[pa]);
				cv::Point3_<dType> cameraPtB = imgs.at(img.second.imageId).worldPtInView(meshPts[pb]);
				cv::Point3_<dType> cameraPtC = imgs.at(img.second.imageId).worldPtInView(meshPts[pc]);
				cv::Point_<dType> ptReprojA = cameras.at(imgs.at(img.second.imageId).cameraId).ptInView(cameraPtA);
				cv::Point_<dType> ptReprojB = cameras.at(imgs.at(img.second.imageId).cameraId).ptInView(cameraPtB);
				cv::Point_<dType> ptReprojC = cameras.at(imgs.at(img.second.imageId).cameraId).ptInView(cameraPtC);
				const dType& sigmaA = sigma[pa];
				const dType& sigmaB = sigma[pb];
				const dType& sigmaC = sigma[pc];

				std::vector<cv::Point>trianglePts{ cv::Point(ptReprojA) ,cv::Point(ptReprojB) ,cv::Point(ptReprojC) };
				cv::drawContours(mask, std::vector<std::vector<cv::Point>>{ trianglePts }, 0, cv::Scalar(255), -1);
				cv::Rect bbos = cv::boundingRect(trianglePts);
				cv::Mat local = cv::Mat::zeros(bbos.height, bbos.width, CV_8UC1);
				cv::Mat localDepth = cv::Mat::zeros(bbos.height, bbos.width, CV_32FC1);
				for (auto&d: trianglePts)
				{
					d.x -= bbos.x;
					d.y -= bbos.y;
				}
				cv::drawContours(local, std::vector<std::vector<cv::Point>>{ trianglePts }, 0, cv::Scalar(255), -1);
				dType y2_y3 = trianglePts[1].y - trianglePts[2].y;
				dType x3_x2 = trianglePts[2].x - trianglePts[1].x;
				dType x1_x3 = trianglePts[0].x - trianglePts[2].x;
				dType y1_y3 = trianglePts[0].y - trianglePts[2].y;
				dType y3_y1 = trianglePts[2].y - trianglePts[0].y;  
				dType denom = y2_y3 * x1_x3 + x3_x2 * y1_y3;
//#pragma omp parallel for
				for (int r_ = 0; r_ < bbos.height; r_++)
				{
					for (int c_ = 0; c_ < bbos.width; c_++)
					{
						if (local.ptr<uchar>(r_)[c_]>0)
						{
							int trueR = r_ + bbos.y;
							int trueC = c_ + bbos.x;
							if (trueR < 0 || trueC < 0 || trueR >= pixelMesh.rows || trueC >= pixelMesh.cols)
							{
								continue;
							}
							dType px_x3 = c_ - trianglePts[2].x;
							dType py_y3 = r_ - trianglePts[2].y;
							dType w0 = (y2_y3 * px_x3 + x3_x2 * py_y3) ;
							dType w1 = (y3_y1 * px_x3 + x1_x3 * py_y3) ;
							dType w2 = 1 - w0 - w1;
							if (abs(denom)<1e-5)
							{
								w0 = 1. / 3;
								w1 = w0; w2 = w0;
							}
							else
							{
								w0/=denom;
								w1/=denom;
								w2 = 1 - w0 - w1;
							}
							localDepth.ptr<float>(r_)[c_] = w0 * cameraPtA.z + w1 * cameraPtB.z + w2 * cameraPtC.z;
							dType theSigma = w0 * sigmaA + w1 * sigmaB + w2 * sigmaC;
							bool a1 = pixelMesh.ptr<int>(trueR)[trueC] >= 0;
							bool a2 = pixelDist.ptr<float>(trueR)[trueC] < localDepth.ptr<float>(r_)[c_];
							if (!(a1 && a2))
							{
								pixelDist.ptr<float>(trueR)[trueC] = localDepth.ptr<float>(r_)[c_];
								pixelSigma.ptr<float>(trueR)[trueC] = theSigma;
								pixelMesh.ptr<int>(trueR)[trueC] = f;
							} 
						}
					}
				}
			}
		}
		LOG(INFO);
	}
	SelectNeighborViews(cameras, imgs, objPts);





	std::vector<int>imgKeys;
	imgKeys.reserve(imgs.size());
	for (auto&d: imgs)
	{
		imgKeys.emplace_back(d.first);
	}
	std::sort(imgKeys.begin(), imgKeys.end());
	LOG(INFO) << "img size = " << imgKeys.size();
	cv::Mat imgFirst = cv::imread("viewer/"+imgs[imgKeys[0]].imgPath);
	CHECK(!imgFirst.empty());
	std::vector<cv::Point_<dType>>standardImgPts = getStandardImgPts(imgFirst.size());
	std::vector<cv::Point_<dType>>standardCameraPts(standardImgPts.size());
#pragma omp parallel for
	for (int p = 0; p < standardImgPts.size(); p++)
	{
		standardCameraPts[p] = cameras[0].pixel2cam(standardImgPts[p]);
	}
	for (size_t i = 0; i < imgKeys.size()-1; i++)
	{
		std::string imgAPath = imgs[imgKeys[i]].imgPath;
		std::string imgBPath = imgs[imgKeys[i + 1]].imgPath;
		std::string imgBFlowPath ="RAFT/"+ imgAPath + "-" + imgBPath + ".bin";
		std::vector<cv::Point_<dType>>flowImgPts = getFlowImgPts(standardImgPts, imgBFlowPath);
		std::vector<cv::Point_<dType>>flowCameraPts(flowImgPts.size());
#pragma omp parallel for
		for (int p = 0; p < standardImgPts.size(); p++)
		{
			flowCameraPts[p] = cameras[0].pixel2cam(flowImgPts[p]);
		}
		cv::Mat_<dType> pts;
		cv::triangulatePoints(imgs[imgKeys[i]].worldToCamera, imgs[imgKeys[i + 1]].worldToCamera, standardCameraPts, flowCameraPts, pts);
		writePts("RAFT/" + imgAPath + ".pts", pts);
	}
	//
	return 0;
}