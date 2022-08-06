extern "C" {
#include "shared/shared.h"
#include "common/bsp.h"
}

#include "common/bsp_util.h"

#undef clamp //Stop learning from Windows.h people! ffs!

#include <vector>
#include <functional>

EXPORT typedef void (*BSP_ReceiveLeafsFPtr) (bsp_t* bsp, mleaf_t* leaf, void* context);
EXPORT typedef void (*BSP_ReceiveBrushLeafFPtr) (bsp_t* bsp, mmodel_t* leaf, mbrush_t* brush, void* context);
EXPORT typedef void (*BSP_ReceiveBrushPlaneFPtr) (bsp_t* bsp, mmodel_t* leaf, mbrush_t* brush, cplane_t* plane, float plane_nx, float plane_ny, float plane_nz, float plane_d, void* context);

// void (*G_Passthrough_BSP_Loaded)(bsp_t* bsp) = nullptr;
// void (*G_Passthrough_BSP_Destroy)(bsp_t* bsp) = nullptr;

void BSP_RecurseLeafs(bsp_t* bsp, mnode_t* node, void* context, BSP_ReceiveLeafsFPtr callback)
{
    if(!node)
    {
        return;
    }

	for (int i = 0; i < 2; ++i)
	{
		if (node->plane == nullptr)
        {
            callback(bsp, (mleaf_t*)node, context);
        }
		else
			BSP_RecurseLeafs(bsp, node->children[i], context, callback);
	}
}

void BSP_RecurseBrushes(bsp_t* bsp, mmodel_t* root, int brushMask, void* context, BSP_ReceiveBrushLeafFPtr callback)
{
	// call callback for each brush
    struct BrushRecursionContext_T
    {
        void* context;
        BSP_ReceiveBrushLeafFPtr callback;
        int brushMask;
        mmodel_t* root;
        std::vector<mbrush_t*> brushesDone;
    };

    BrushRecursionContext_T brushContext {context, callback, brushMask, root};

	BSP_RecurseLeafs(
        bsp, root->headnode, &brushContext,
        +[](bsp_t* bsp, mleaf_t* leaf, void* context){

            BrushRecursionContext_T& brushContext = *(BrushRecursionContext_T*)context;

            for (int b=0; b<leaf->numleafbrushes; b++)
            {
                mbrush_t* brush = leaf->firstleafbrush[b];
                if (std::find(brushContext.brushesDone.begin(), brushContext.brushesDone.end(), brush) == brushContext.brushesDone.end())
                {
                    if (brush->contents & brushContext.brushMask)
                    {
                        brushContext.brushesDone.push_back(brush);

                        brushContext.callback(bsp, brushContext.root, brush, brushContext.context);
                    }
                }
            }
        }
    );
}

void BSP_RecurseBrushPlanes(bsp_t* bsp, void* context, int brushMask, BSP_ReceiveBrushPlaneFPtr callback)
{
    struct BrushRecursionContext_T
    {
        void* context;
        BSP_ReceiveBrushPlaneFPtr callback;
    };

    BrushRecursionContext_T planeContext {context, callback};

    for(size_t m = 0; m < bsp->nummodels; ++m)
    {
        BSP_RecurseBrushes(
            bsp, bsp->models + m, brushMask, &planeContext,
            [](bsp_t* bsp, mmodel_t* leaf, mbrush_t* brush, void* context)
            {
                BrushRecursionContext_T& planeContext = *(BrushRecursionContext_T*)context;

                for (int p=0; p<brush->numsides; p++)
                {
                    mbrushside_t& brushside = brush->firstbrushside[p];
                    cplane_t* plane = brushside.plane;

                    planeContext.callback(
                        bsp, leaf, brush, plane,
                        plane->normal[0], plane->normal[1], plane->normal[2], -plane->dist,
                        planeContext.context
                    );
                }
            }
        );
    }
}

/*
	std::vector<mbrush_t*> brushesAdded;
	for (size_t i = 0; i < leafs.size(); i++)
	{
		bool isValidBrush = false;

		mleaf_t& leaf = *(mleaf_t*)leafs[i];

		for (int b=0; b<leaf.numleafbrushes; b++)
		{
			btAlignedObjectArray<btVector3> planeEquations;

			mbrush_t* brush = leaf.firstleafbrush[b];

			bool has = false;
			for (size_t x = 0; x < brushesAdded.size(); ++x)
			{
				if (brushesAdded[x] == brush)
				{
					has = true;
					break;
				}
			}

			if (!has)
			{
				if (brush->contents & (CONTENTS_SOLID|CONTENTS_WINDOW))
				{
					brushesAdded.push_back(brush);

					for (int p=0; p<brush->numsides; p++)
					{
						mbrushside_t& brushside = brush->firstbrushside[p];
						cplane_t* plane = brushside.plane;
						btVector3 planeEq;
						planeEq.setValue(
							plane.normal[0],
							plane.normal[1],
							plane.normal[2]);
						planeEq[3] = -plane.dist;

						planeEquations.push_back(planeEq);
						isValidBrush = true;
					}

					if (isValidBrush)
					{
						btAlignedObjectArray<btVector3>	vertices;
						btGeometryUtil::getVerticesFromPlaneEquations(planeEquations, vertices);

						receiveVertices(vertices, model, index++, context);
					}
				}
			} 
		}
	}
}
*/