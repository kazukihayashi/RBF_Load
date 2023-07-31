using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace RBF_Load
{
    public class RBF_LoadInfo : GH_AssemblyInfo
    {
        public override string Name => "RBF Load";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => Properties.Resources.RBF_load_icon;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "This component approximates the target shape with a funicular geometry by load control using RBF interpolation.";

        public override Guid Id => new Guid("8C4D62C0-7289-4941-82DB-C2DFB11AB0B2");

        //Return a string identifying you or your company.
        public override string AuthorName => "Kazuki Hayashi";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "hayashi.kazuki.55a@gmail.com";
    }
}