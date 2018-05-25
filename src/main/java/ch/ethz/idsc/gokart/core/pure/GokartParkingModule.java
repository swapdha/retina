// code by ynager and jph
package ch.ethz.idsc.gokart.core.pure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.gokart.core.pos.GokartPoseEvent;
import ch.ethz.idsc.gokart.core.pos.GokartPoseHelper;
import ch.ethz.idsc.gokart.core.pos.GokartPoseLcmClient;
import ch.ethz.idsc.gokart.core.pos.GokartPoseListener;
import ch.ethz.idsc.gokart.core.slam.PredefinedMap;
import ch.ethz.idsc.gokart.lcm.autobox.RimoGetLcmClient;
import ch.ethz.idsc.gokart.lcm.mod.PlannerPublish;
import ch.ethz.idsc.owl.bot.r2.ImageCostFunction;
import ch.ethz.idsc.owl.bot.r2.ImageEdges;
import ch.ethz.idsc.owl.bot.r2.ImageRegions;
import ch.ethz.idsc.owl.bot.se2.Se2CarIntegrator;
import ch.ethz.idsc.owl.bot.se2.Se2ComboRegion;
import ch.ethz.idsc.owl.bot.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owl.bot.se2.Se2PointsVsRegions;
import ch.ethz.idsc.owl.bot.se2.Se2Wrap;
import ch.ethz.idsc.owl.bot.se2.glc.CarFlows;
import ch.ethz.idsc.owl.bot.util.FlowsInterface;
import ch.ethz.idsc.owl.car.core.VehicleModel;
import ch.ethz.idsc.owl.car.shop.RimoSinusIonModel;
import ch.ethz.idsc.owl.data.Lists;
import ch.ethz.idsc.owl.glc.adapter.Expand;
import ch.ethz.idsc.owl.glc.adapter.GlcTrajectories;
import ch.ethz.idsc.owl.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owl.glc.adapter.RegionConstraints;
import ch.ethz.idsc.owl.glc.adapter.Trajectories;
import ch.ethz.idsc.owl.glc.core.CostFunction;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.PlannerConstraint;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.math.StateTimeTensorFunction;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.retina.sys.AbstractModule;
import ch.ethz.idsc.retina.util.math.Magnitude;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.io.ResourceData;
import ch.ethz.idsc.tensor.qty.Degree;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.red.Entrywise;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Sqrt;

public class GokartParkingModule extends AbstractModule implements GokartPoseListener {
  private static final VehicleModel STANDARD = RimoSinusIonModel.standard();
  // TODO make configurable as parameter
  private static final Tensor PARTITIONSCALE = Tensors.of( //
      RealScalar.of(10), RealScalar.of(10), Degree.of(5).reciprocal()).unmodifiable();
  private static final Scalar SQRT2 = Sqrt.of(RealScalar.of(2));
  private static final Scalar SPEED = RealScalar.of(2.5);
  private static final FixedStateIntegrator FIXEDSTATEINTEGRATOR = // node interval == 2/5
      FixedStateIntegrator.create(Se2CarIntegrator.INSTANCE, RationalScalar.of(2, 10), 4);
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 1));
  // ---
  final FlowsInterface carFlows = CarFlows.forward( //
      SPEED, Magnitude.PER_METER.apply(TrajectoryConfig.GLOBAL.maxRotation));
  private final GokartPoseLcmClient gokartPoseLcmClient = new GokartPoseLcmClient();
  private final RimoGetLcmClient rimoGetLcmClient = new RimoGetLcmClient();
  private final Collection<CostFunction> costCollection = new LinkedList<>();
  final PurePursuitModule purePursuitModule = new PurePursuitModule();
  private final Region<Tensor> fixedRegion;
  private GokartPoseEvent gokartPoseEvent = null;
  public List<TrajectorySample> trajectory = null;
  private final Tensor obstacleMap;
  private final PlannerConstraint plannerConstraint;
  private final Tensor goalRadius;
  private Tensor parkingPose = Tensors.vector(30, 40, -2.35);

  public GokartParkingModule() {
    PredefinedMap predefinedMap = PredefinedMap.DUBENDORF_HANGAR_20180423OBSTACLES;
    obstacleMap = ImageRegions.grayscale(ResourceData.of("/map/dubendorf/hangar/20180423obstacles.png"));
    Tensor hull = STANDARD.footprint();
    Tensor min = hull.stream().reduce(Entrywise.min()).get(); // {-0.295, -0.725, -0.25}
    Tensor max = hull.stream().reduce(Entrywise.max()).get(); // {1.765, 0.725, -0.25}
    int ttl = Ceiling.of(max.Get(1).multiply(predefinedMap.scale())).number().intValue(); // == 0.73 * 7.5 == 5.475 => 6
    Tensor tensor = ImageEdges.extrusion(obstacleMap, ttl);
    ImageRegion imageRegion = predefinedMap.getImageRegion();
    Tensor x_samples = Subdivide.of(min.get(0), max.get(0), 2); // {-0.295, 0.7349999999999999, 1.765}
    fixedRegion = Se2PointsVsRegions.line(x_samples, imageRegion);
    System.out.println(predefinedMap.getImage().getHeight());
    // ---
    costCollection.add(ImageCostFunction.of(tensor, predefinedMap.range(), RealScalar.ZERO));
    // ---
    final Scalar goalRadius_xy = SQRT2.divide(PARTITIONSCALE.Get(0));
    final Scalar goalRadius_theta = SQRT2.divide(PARTITIONSCALE.Get(2));
    goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
    plannerConstraint = RegionConstraints.timeInvariant(fixedRegion);
  }

  @Override // from AbstractClockedModule
  protected void first() throws Exception {
    gokartPoseLcmClient.addListener(this);
    // ---
    gokartPoseLcmClient.startSubscriptions();
    rimoGetLcmClient.startSubscriptions();
    // ---
    purePursuitModule.launch();
    // ---
    while(Objects.isNull(gokartPoseEvent))
      Thread.sleep(100);
    System.out.println("Calculating parking trajectory...");
    final Tensor xya = GokartPoseHelper.toUnitless(gokartPoseEvent.getPose()).unmodifiable();
    StateTime stateTime = new StateTime(xya, RealScalar.ZERO);
    final List<TrajectorySample> head = Arrays.asList(TrajectorySample.head(stateTime));
    // ---
   // Tensor goal = Tensors.vector(225/7.5, 340/7.5, -2.35);
    int resolution = TrajectoryConfig.GLOBAL.controlResolution.number().intValue();
    Collection<Flow> controls = carFlows.getFlows(resolution);
    Se2ComboRegion se2ComboRegion = //
        Se2ComboRegion.spherical(parkingPose, goalRadius);
    GoalInterface goalInterface = new Se2MinTimeGoalManager(se2ComboRegion, controls).getGoalInterface();
    GoalInterface multiCostGoalInterface = MultiCostGoalAdapter.of(goalInterface, costCollection);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        PARTITIONSCALE, FIXEDSTATEINTEGRATOR, controls, plannerConstraint, multiCostGoalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(SE2WRAP::represent);
    // Do Planning
    StateTime root = Lists.getLast(head).stateTime(); // non-empty due to check above
    trajectoryPlanner.insertRoot(root);
    Expand.maxTime(trajectoryPlanner, Quantity.of(5, SI.SECOND)); // TODO magic
    expandResult(head, trajectoryPlanner); // build detailed trajectory and pass to purePursuit
    return;
  }

  @Override // from AbstractClockedModule
  protected void last() {
    purePursuitModule.terminate();
    gokartPoseLcmClient.stopSubscriptions();
    // ---
  }

  @Override // from GokartPoseListener
  public void getEvent(GokartPoseEvent gokartPoseEvent) { // arrives at 50[Hz]
    this.gokartPoseEvent = gokartPoseEvent;
  }

  public void expandResult(List<TrajectorySample> head, TrajectoryPlanner trajectoryPlanner) {
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) { // goal reached
      List<TrajectorySample> tail = //
          GlcTrajectories.detailedTrajectoryTo(trajectoryPlanner.getStateIntegrator(), optional.get());
      trajectory = Trajectories.glue(head, tail);
      Tensor curve = Tensor.of(trajectory.stream().map(ts -> ts.stateTime().state().extract(0, 2)));
      purePursuitModule.setCurve(Optional.of(curve));
      PlannerPublish.publishTrajectory(trajectory);
    } else {
      // failure to reach goal
      purePursuitModule.setCurve(Optional.empty());
      PlannerPublish.publishTrajectory(new ArrayList<>());
    }
  }
}
