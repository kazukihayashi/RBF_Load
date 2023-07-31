using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Kangaroo2Component;
using MathNet.Numerics.LinearAlgebra;
using Grasshopper.Kernel.Data;
using System.Linq;

namespace RBF_Load
{
    public class RBF_LoadComponent : GH_Component
    {
        int totalIter;
        const double multiplicationFactor = 10;
        Func<double, double, double> RBF = (x, b) => Math.Exp(-Math.Pow(b * x, 2));

        private Kangaroo2Component.GoalComponents.GeometryComponent showGeometryComponent;
        private Kangaroo2Component.GoalComponents.SpringByLineComponent lengthComponent;
        private Kangaroo2Component.GoalComponents.Anchor2Component anchorComponent;
        private Kangaroo2Component.GoalComponents.AnchorXYZComponent anchorXYZComponent;
        private Kangaroo2Component.GoalComponents.UnaryComponent loadComponent;
        private KangarooZombie zombieSolverComponent;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public RBF_LoadComponent()
          : base("RBF Load", "RBF Load",
            "This component approximates the target shape with a funicular geometry by load control using RBF interpolation.",
            "Extra", "Extra")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("mesh", "M", "Input mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("fix", "Pf", "Fixed points", GH_ParamAccess.list);
            pManager.AddPointParameter("target", "Pt", "Target points", GH_ParamAccess.list);
            pManager.AddIntegerParameter("I", "I", "Number of iterations to modify the shape", GH_ParamAccess.item, 10);
            pManager.AddNumberParameter("beta", "b", "RBF parameter", GH_ParamAccess.item, 1e-5);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("mesh", "M", "Output mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("point", "P", "Output Points", GH_ParamAccess.list);
            pManager.AddVectorParameter("load", "V", "Load vectors", GH_ParamAccess.list);
            pManager.AddNumberParameter("RMSE", "e", "Root mean square error (RMSE) of the target shape approximation.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("I", "I", "Total steps to compute equilibrium shapes. This is equivalent to the sum of the numbers of iterations that the Kangaroo solver outputs at each solving.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double variance = 0.0;
            double loadFactor = 0.01;
            totalIter = 0;

            Mesh mesh_input = new Mesh();
            var pt_fix = new List<Point3d>();
            var pt_target = new List<Point3d>();
            int maxIter = 0;
            double beta = 0;

            DA.GetData(0, ref mesh_input);
            DA.GetDataList(1, pt_fix);
            DA.GetDataList(2, pt_target);
            DA.GetData(3, ref maxIter);
            DA.GetData(4, ref beta);

            // Retrieve mesh nodes and lines
            var pt_in = new Rhino.Geometry.PointCloud(mesh_input.Vertices.ToPoint3dArray());
            var edge_in = ExtractMeshEdges(mesh_input);
            var edge_length_in = edge_in.Select(e => e.Length).ToArray();
            var pt_fix_i = pt_fix.Select(pf => pt_in.ClosestPoint(pf)).ToArray();

            // Input matrix for RBF interpolation

            Matrix<double> matA = MatA(pt_target.ToArray(), beta);

            // variable objects
            var mesh_out = mesh_input.DuplicateMesh();
            var loadVec_temp = Enumerable.Repeat(new Vector3d(), pt_in.Count).ToArray();
            var loadVec_out = Enumerable.Repeat(new Vector3d(), pt_in.Count).ToArray();

            var loadFac_history = new double[maxIter];

            // iterate
            for (int iter = 0; iter < maxIter; iter++)
            {
                // mesh geometry
                showGeometryComponent = new Kangaroo2Component.GoalComponents.GeometryComponent();
                showGeometryComponent.Params.Input[0].AddVolatileData(new GH_Path(0), 0, mesh_input);
                showGeometryComponent.CollectData();
                showGeometryComponent.ComputeData();

                // springs
                lengthComponent = new Kangaroo2Component.GoalComponents.SpringByLineComponent();
                lengthComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), edge_in);
                lengthComponent.Params.Input[1].AddVolatileDataList(new GH_Path(0), edge_length_in);
                //lengthComponent.Params.Input[2].AddVolatileData(new GH_Path(0), 0, 10.0);
                lengthComponent.CollectData();
                lengthComponent.ComputeData();

                // fix at supports
                anchorComponent = new Kangaroo2Component.GoalComponents.Anchor2Component();
                anchorComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), pt_fix);
                anchorComponent.Params.Input[1].AddVolatileDataList(new GH_Path(0), pt_fix);
                anchorComponent.Params.Input[2].AddVolatileData(new GH_Path(0), 0, 10000);
                anchorComponent.CollectData();
                anchorComponent.ComputeData();

                // fix in x,y directions only
                anchorXYZComponent = new Kangaroo2Component.GoalComponents.AnchorXYZComponent();
                anchorXYZComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), pt_in.GetPoints());
                anchorXYZComponent.Params.Input[1].AddVolatileData(new GH_Path(0), 0, true); // fix in x-direction
                anchorXYZComponent.Params.Input[2].AddVolatileData(new GH_Path(0), 0, true); // fix in y-direction
                //anchorXYZComponent.Params.Input[4].AddVolatileData(new GH_Path(0), 0, 1000);
                anchorXYZComponent.CollectData();
                anchorXYZComponent.ComputeData();

                var dpZ_unscaled = LoadZ(mesh_out, pt_target, beta, matA);

                // increase loadFactor?
                for (int i = 0; i < pt_in.Count; i++)
                {
                    loadVec_temp[i].Z = loadVec_out[i].Z + dpZ_unscaled[i] * loadFactor * multiplicationFactor;
                }
                var mesh_temp_inc = EquilibriumShape(pt_in.GetPoints(), loadVec_temp);
                double variance_inc = Zdiff(mesh_temp_inc, pt_target).Select(x => x * x).Sum();

                // decrease loadFactor?
                for (int i = 0; i < pt_in.Count; i++)
                {
                    loadVec_temp[i].Z = loadVec_out[i].Z + dpZ_unscaled[i] * loadFactor * 1 / multiplicationFactor;
                }
                var mesh_temp_dec = EquilibriumShape(pt_in.GetPoints(), loadVec_temp);
                double variance_dec = Zdiff(mesh_temp_dec, pt_target).Select(x => x * x).Sum();

                // keep loadFactor?
                for (int i = 0; i < pt_in.Count; i++)
                {
                    loadVec_temp[i].Z = loadVec_out[i].Z + dpZ_unscaled[i] * loadFactor;
                }
                var mesh_temp_keep = EquilibriumShape(pt_in.GetPoints(), loadVec_temp);
                double variance_keep = Zdiff(mesh_temp_keep, pt_target).Select(x => x * x).Sum();

                if (variance_keep < variance_dec && variance_keep < variance_inc) // keep loadFactor
                {
                    variance = variance_keep;
                    mesh_out = mesh_temp_keep.DuplicateMesh();
                    Array.Copy(loadVec_temp, loadVec_out, loadVec_out.Length);
                }
                else
                {
                    double scaler = 0;
                    double variance_before = 0.0;
                    if (variance_dec <= variance_inc) // decrease loadFactor
                    {
                        scaler = 1 / multiplicationFactor;
                        variance_before = variance_dec;
                        mesh_out = mesh_temp_dec.DuplicateMesh();
                    }
                    else // increase loadFactor
                    {
                        scaler = multiplicationFactor;
                        variance_before = variance_inc;
                        mesh_out = mesh_temp_inc.DuplicateMesh();
                    }
                    loadFactor *= scaler;
                    while (true)
                    {
                        // load
                        for (int i = 0; i < pt_in.Count; i++)
                        {
                            loadVec_temp[i].Z = loadVec_out[i].Z + dpZ_unscaled[i] * loadFactor * scaler;
                        }

                        var mesh_temp = EquilibriumShape(pt_in.GetPoints(), loadVec_temp);

                        var zdiff = Zdiff(mesh_temp, pt_target);
                        variance = zdiff.Select(x => x * x).Sum();
                        if (variance < variance_before)
                        {
                            loadFactor *= scaler;
                            variance_before = variance;
                            mesh_out = mesh_temp.DuplicateMesh();
                        }
                        else
                        {
                            variance = variance_before;
                            break;
                        }
                    }
                    // load
                    for (int i = 0; i < pt_in.Count; i++)
                    {
                        loadVec_temp[i].Z = loadVec_out[i].Z + dpZ_unscaled[i] * loadFactor;
                    }
                    Array.Copy(loadVec_temp, loadVec_out, loadVec_out.Length);
                }
                loadFac_history[iter] = loadFactor;
            }

            //mesh_out.Vertices.Clear();
            //mesh_out.Vertices.AddVertices();

            DA.SetData(0, mesh_out);
            DA.SetDataList(1, mesh_out.Vertices.ToPoint3dArray());
            DA.SetDataList(2, loadVec_out);
            DA.SetData(3, Math.Sqrt(variance / pt_target.Count));
            DA.SetData(4, totalIter);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => Properties.Resources.RBF_load_icon;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("AB4D6903-4659-471C-86FF-8B8445398703");

        private Line[] ExtractMeshEdges(Mesh mesh)
        {
            Line[] edges = new Line[mesh.TopologyEdges.Count];
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                edges[i] = mesh.TopologyEdges.EdgeLine(i);
            }
            return edges;
        }

        private Mesh EquilibriumShape(Point3d[] pt, Vector3d[] loadVec)
        {
            // load component
            loadComponent = new Kangaroo2Component.GoalComponents.UnaryComponent();
            loadComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), pt);
            loadComponent.Params.Input[1].AddVolatileDataList(new GH_Path(0), loadVec);
            loadComponent.CollectData();
            loadComponent.ComputeData();

            // solver
            zombieSolverComponent = new KangarooZombie();
            zombieSolverComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), showGeometryComponent.Params.Output[0].VolatileData.AllData(true));
            zombieSolverComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), lengthComponent.Params.Output[0].VolatileData.AllData(true));
            zombieSolverComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), anchorComponent.Params.Output[0].VolatileData.AllData(true));
            zombieSolverComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), anchorXYZComponent.Params.Output[0].VolatileData.AllData(true));
            zombieSolverComponent.Params.Input[0].AddVolatileDataList(new GH_Path(0), loadComponent.Params.Output[0].VolatileData.AllData(true));

            zombieSolverComponent.CollectData();
            zombieSolverComponent.ComputeData();

            totalIter += ((Grasshopper.Kernel.Types.GH_Integer)zombieSolverComponent.Params.Output[0].VolatileData.AllData(true).First()).Value;
            Mesh outMesh = ((Grasshopper.Kernel.Types.GH_Mesh)zombieSolverComponent.Params.Output[2].VolatileData.AllData(true).First()).Value.DuplicateMesh();

            return outMesh;
        }

        private double[] Zdiff(Mesh mesh, List<Point3d> pt)
        {
            ///
            /// if positive, the point is above the mesh
            /// if negative, the point is below the mesh
            ///
            double[] zdiff = new double[pt.Count];
            var pt_project_to_mesh = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(new Mesh[1] { mesh }, pt, Vector3d.ZAxis, 1e-8);
            pt_project_to_mesh = Rhino.Geometry.Point3d.CullDuplicates(pt_project_to_mesh, 1e-5);
            if (pt_project_to_mesh.Length != pt.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Some target points could not be projected onto the mesh in the z-direction.");
                for (int i = 0; i < pt.Count; i++)
                {
                    zdiff[i] = 1e20;
                }
            }
            else
            {
                for (int i = 0; i < pt.Count; i++)
                {
                    zdiff[i] = pt[i].Z - pt_project_to_mesh[i].Z;
                }
            }

            return zdiff;
        }

        private double[] LoadZ(Mesh mesh, List<Point3d> pt_target, double beta, Matrix<double> matA)
        {
            var pt_in_2D = mesh.Vertices.ToPoint3dArray();
            var pt_target_2D = pt_target.ToArray();
            for (int i = 0; i < pt_in_2D.Length; i++)
            {
                pt_in_2D[i].Z = 0;
            }
            for (int i = 0; i < pt_target_2D.Length; i++)
            {
                pt_target_2D[i].Z = 0;
            }

            var vec_b = Vector<double>.Build.DenseOfArray(Zdiff(mesh, pt_target));

            var weight = matA.Solve(vec_b);
            var pZ = new double[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                for (int j = 0; j < pt_target.Count; j++)
                {
                    pZ[i] += weight[j] * RBF(pt_in_2D[i].DistanceTo(pt_target_2D[j]), beta);
                }
            }

            return pZ;
        }

        private Matrix<double> MatA(Point3d[] pt_target, double beta)
        {
            Point3d[] pt_target_2D = pt_target.ToArray();
            for (int i = 0; i < pt_target_2D.Length; i++)
            {
                pt_target_2D[i].Z = 0;
            }

            // matrix A
            Matrix<double> matA = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.Dense(pt_target_2D.Length, pt_target_2D.Length);
            for (int i = 0; i < pt_target_2D.Length - 1; i++)
            {
                for (int j = i + 1; j < pt_target_2D.Length; j++)
                {
                    matA[i, j] = RBF(pt_target_2D[i].DistanceTo(pt_target_2D[j]), beta);
                    matA[j, i] = matA[i, j];
                }
            }
            for (int i = 0; i < pt_target_2D.Length; i++)
            {
                matA[i, i] = RBF(pt_target_2D[i].DistanceTo(pt_target_2D[i]), beta);
            }

            return matA;
        }

    }
}