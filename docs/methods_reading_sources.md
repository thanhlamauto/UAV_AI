# Giai thich phuong phap va nguon doc cho report

Tai lieu nay tom tat cac phuong phap dang xuat hien trong report UAV ODA, kem nguon doc nen trich dan. Khi dua vao bao cao, nen uu tien cac paper goc, trang dataset chinh thuc, va official documentation.

## 1. ODA Dataset va ground-truth trajectory

**Dung trong project:** ODA la benchmark chinh cho bai toan MAV indoor obstacle avoidance. Pipeline doc `trial_overview.csv`, `optitrack.csv`, `radar.csv`, `imu.csv` va video RGB de tai dung quy dao, vat can, risk label va video dinh tinh.

**Giai thich ngan:** Dataset nay phu hop vi no co MAV bay trong nha/GNSS-denied, co camera/radar/IMU va OptiTrack ground truth. OptiTrack cho phep tinh khoang cach that giua MAV va vat can tren mat phang bay.

**Nguon doc:**

- ODA GitHub: https://github.com/JuSquare/ODA_Dataset
- ODA 4TU record/DOI: https://doi.org/10.4121/14214236.v1
- ODA paper page: https://research.tudelft.nl/en/publications/a-novel-obstacle-detection-and-avoidance-dataset-for-drones/
- ODA paper PDF: https://research.tudelft.nl/files/125576195/3522784.3522786.pdf

## 2. Safety clearance, collision va risk label

**Dung trong project:** Moi vat can duoc mo hinh hoa thanh hinh tron/hinh tru tren mat phang bay. Clearance = khoang cach MAV den tam vat can tru ban kinh vat can. Neu clearance < safety distance thi tinh la safety-distance violation; neu clearance < 0 thi collision.

**Giai thich ngan:** Day la obstacle representation hinh hoc toi thieu nhung ro rang: thay vi phai dung detector nang, ta dung ground truth de danh gia quy dao co di qua vung nguy hiem hay khong. Risk label `safe/warning/danger/collision` duoc tao tu clearance.

**Nguon doc nen doc nen:**

- Steven LaValle, *Planning Algorithms*, motion planning and collision checking: https://lavalle.pl/planning/
- Path planning evaluation context: https://www.nature.com/articles/s41598-025-96614-2

## 3. Occupancy grid va A* planner

**Dung trong project:** A* bien mat phang bay thanh luoi 2D. Cac o nam trong obstacle+safety radius bi danh dau occupied. A* tim duong co chi phi nho tu start den goal.

**Giai thich ngan:** A* la graph-search planner co heuristic, thuong dung khi moi truong da duoc roi rac hoa thanh grid. Trong project, A* la baseline classical planner de so sanh voi quy dao nguoi lai va sampling-based planners.

**Nguon doc:**

- Hart, Nilsson, Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths": https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf
- LaValle, *Planning Algorithms*: https://lavalle.pl/planning/

## 4. RRT va RRT*

**Dung trong project:** RRT lay mau ngau nhien trong khong gian bay 2D, noi mau moi voi nut gan nhat neu doan noi khong cat vat can. RRT* bo sung buoc chon parent/rewire de toi uu dan chi phi duong di.

**Giai thich ngan:** RRT/RRT* thuoc nhom sampling-based planning. RRT manh khi khong gian lon va co vat can, nhung duong di co the khong toi uu. RRT* cham hon nhung co tinh chat asymptotic optimality, nghia la khi tang so mau, loi giai co xu huong tien den duong toi uu.

**Nguon doc:**

- LaValle, "Rapidly-exploring random trees: A new tool for path planning": https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
- LaValle RRT publication list/explanation: https://lavalle.pl/rrtpubs.html
- Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning" / RRT*: https://arxiv.org/abs/1105.1186

## 5. MPPI va MPC

**Dung trong project:** MPPI Python nhe lay nhieu rollout ung vien, cham diem bang cost gom goal distance, obstacle penalty, path length, smoothness va collision penalty, roi cap nhat trajectory theo rollout tot.

**Giai thich ngan:** MPC la dieu khien toi uu theo cua so truot: tai moi thoi diem giai bai toan toi uu tren horizon ngan roi ap dung control dau tien. MPPI la bien the sampling-based optimal control: khong can dao ham phuc tap, phu hop voi cost phi tuyen va penalty va cham.

**Nguon doc:**

- Williams et al., "Information Theoretic Model Predictive Control": https://arxiv.org/abs/1707.02342
- Williams, Aldrich, Theodorou, "Model Predictive Path Integral Control: From Theory to Parallel Computation": https://arc.aiaa.org/doi/pdf/10.2514/1.G001921
- Mayne et al., "Constrained model predictive control: Stability and optimality": https://dl.acm.org/doi/10.1016/S0005-1098%2899%2900214-9
- Rapyuta MPPI controller reference repo: https://github.com/rapyuta-robotics/mppi_controller

## 6. Monocular predicted depth: DPT/MiDaS va Depth Anything V2

**Dung trong project:** Depth panel sinh relative depth tu RGB frame. Feature risk dung `depth_min`, `depth_p10`, `depth_median` trong center crop. Report can noi ro day la relative depth, chua phai metric depth theo met neu chua hieu chuan voi ground truth/radar.

**Giai thich ngan:** Monocular depth estimation du doan do sau tu mot anh RGB. Vi mot camera don khong co baseline stereo, model thuong hoc relative/inverse depth; metric depth can fine-tune/hieu chuan rieng.

**Nguon doc:**

- DPT paper, "Vision Transformers for Dense Prediction": https://arxiv.org/abs/2103.13413
- MiDaS paper, "Towards Robust Monocular Depth Estimation": https://arxiv.org/abs/1907.01341
- Intel DPT-Hybrid MiDaS model card: https://huggingface.co/Intel/dpt-hybrid-midas
- PyTorch Hub MiDaS note on relative inverse depth: https://pytorch.org/hub/intelisl_midas_v2/
- Depth Anything V2 paper: https://arxiv.org/abs/2406.09414
- Depth Anything V2 official GitHub: https://github.com/DepthAnything/Depth-Anything-V2
- Depth Anything V2 Small model card: https://huggingface.co/depth-anything/Depth-Anything-V2-Small
- Transformers monocular depth task docs: https://huggingface.co/docs/transformers/en/tasks/monocular_depth_estimation

## 7. Radar FFT va Range-Doppler

**Dung trong project:** ODA radar CSV duoc tom tat bang peak/energy va range-Doppler map. Code lam FFT theo chieu range, sau do Doppler FFT theo chuoi chirp de lay peak range bin, peak Doppler bin va energy.

**Giai thich ngan:** FMCW radar do khoang cach bang tan so beat sau khi tron tin hieu phat/thu. Neu co nhieu chirp trong mot frame, FFT thu hai theo truc chirp cho biet Doppler, tuc van toc tuong doi. Range-Doppler map la bieu dien 2D giua khoang cach va van toc.

**Nguon doc:**

- Texas Instruments, fundamentals of mmWave radar sensors: https://www.ti.com/lit/spyy005
- TI mmWave sensing training: https://www.ti.com/content/dam/videos/external-videos/es-mx/2/3816841626001/5415203482001.mp4/subassets/mmwaveSensing-FMCW-offlineviewing_0.pdf
- MathWorks Range-Doppler Response documentation: https://www.mathworks.com/help/phased/ug/range-doppler-response.html
- FMCW radar and feature maps paper example: https://arxiv.org/html/2503.05629v1

## 8. IMU feature

**Dung trong project:** IMU feature dung norm cua acceleration va gyroscope theo rolling window de minh hoa chuyen dong va lam feature cho risk classifier.

**Giai thich ngan:** IMU khong truc tiep thay vat can, nhung no cho biet UAV dang tang toc/quay manh hay on dinh. Khi ket hop voi depth/radar, IMU giup nhan biet bieu hien dieu khien trong cac pha tranh vat can.

**Nguon doc nen gan voi project:** ODA paper/GitHub mo ta sensor suite, va cac nguon ve sensor fusion/visual-inertial neu can mo rong sau. Trong report hien tai chi can noi IMU la feature bo tro, khong claim la detector.

## 9. Perception-risk classifier va imbalanced labels

**Dung trong project:** Tu depth/radar/IMU/clearance, pipeline tao bang feature va train random forest/logistic regression de du doan `future_risk_label`. Split theo trial ID de tranh leak frame cung trial vao ca train va test. Vi future-risk la lop it, report dung macro-F1, balanced accuracy va risk recall.

**Giai thich ngan:** Day la bai toan classification tren feature da trich. Accuracy don thuan co the danh lua khi lop nguy hiem it; macro-F1 va recall cua lop nguy hiem phu hop hon voi bai toan canh bao an toan.

**Nguon doc:**

- scikit-learn RandomForestClassifier: https://scikit-learn.org/stable/modules/generated/sklearn.ensemble.RandomForestClassifier.html
- scikit-learn LogisticRegression: https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.LogisticRegression.html
- scikit-learn GroupShuffleSplit: https://scikit-learn.org/stable/modules/generated/sklearn.model_selection.GroupShuffleSplit.html
- scikit-learn class weights: https://scikit-learn.org/stable/modules/generated/sklearn.utils.class_weight.compute_class_weight.html
- scikit-learn F1 score: https://scikit-learn.org/stable/modules/generated/sklearn.metrics.f1_score.html
- scikit-learn confusion matrix: https://scikit-learn.org/stable/modules/generated/sklearn.metrics.confusion_matrix.html

## 10. LiDAR point cloud segmentation, clustering va 3D bounding box

**Dung trong project:** ARCO/Multi-LiDAR khong thay ODA lam benchmark chinh. Chung duoc dung de chung minh pipeline co nhanh LiDAR thuc: doc point cloud, downsample/voxel, loai mat dat, clustering, tinh 3D bounding box.

**Giai thich ngan:** LiDAR tao point cloud 3D. De bieu dien vat can, ta thuong loc/giảm mau bang voxel grid, tach ground plane, gom cac diem gan nhau thanh cluster, roi lay min/max theo x-y-z de tao axis-aligned 3D bounding box.

**Nguon doc:**

- PCL VoxelGrid tutorial: https://pointclouds.org/documentation/tutorials/voxel_grid.html
- PCL Euclidean Cluster Extraction tutorial: https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
- PCL Conditional Euclidean Clustering: https://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.html
- DBSCAN original paper: https://cdn.aaai.org/KDD/1996/KDD96-037.pdf

## 11. Trajectory smoothness va dynamic feasibility proxy

**Dung trong project:** Report dung path length, heading-change smoothness, speed proxy va compute time. Day moi la proxy 2D, chua phai dynamic feasibility day du cua quadrotor.

**Giai thich ngan:** Duong tranh vat can khong chi can an toan va ngan; no phai de bay. Smoothness do muc do doi huong/curvature/jerk. Quadrotor that thuong can rang buoc velocity, acceleration, jerk/snap, actuator limit; do do report nen ghi ro smoothness hien tai la chi so proxy.

**Nguon doc:**

- LaValle, *Planning Algorithms*: https://lavalle.pl/planning/
- Path smoothing survey: https://pmc.ncbi.nlm.nih.gov/articles/PMC6165411/
- Mellinger & Kumar, minimum snap trajectory generation for quadrotors: https://robo.fish/wiki/images/d/dd/Trajectory_Generation_and_Control_for_Quadrotors_Daniel_Mellinger.pdf

## 12. ONNX Runtime va TensorRT acceleration

**Dung trong project:** ONNX/TensorRT chi la phan toi uu inference depth, khong thay doi metric benchmark. Report hien tai chi nen claim ONNX Runtime CUDA probe da chay, TensorRT engine that chua claim neu thieu runtime/trtexec.

**Giai thich ngan:** ONNX Runtime dung execution providers de chay model tren CPU/GPU. TensorRT toi uu model deep learning cho GPU NVIDIA bang engine rieng, co the dung FP16/INT8 de tang toc nhung can moi truong runtime dung version.

**Nguon doc:**

- ONNX Runtime CUDA Execution Provider: https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html
- ONNX Runtime Execution Providers overview: https://onnxruntime.ai/docs/execution-providers/
- NVIDIA TensorRT documentation: https://docs.nvidia.com/deeplearning/tensorrt/latest/index.html
- TensorRT Quick Start: https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/quick-start-guide.html

## 13. External datasets: ARCO va Multi-LiDAR Multi-UAV

**Dung trong project:** ODA van la benchmark chinh vi no la MAV obstacle avoidance. ARCO dung lam stress-test LiDAR/radar/IMU tren robot mat dat. Multi-LiDAR dung cho huong mo rong UAV tracking/LiDAR, nhung can quyen tai data.

**Giai thich ngan:** Khong nen tron metric ODA va ARCO thanh mot bang benchmark, vi vehicle dynamics va task khac nhau. Nen dung external dataset de chung minh sensing/generalization pressure.

**Nguon doc:**

- ARCO Dataset official page: https://robotics.upo.es/datasets/ArcoDataset/main.html
- Multi-LiDAR Multi-UAV official page: https://tiers.github.io/multi_lidar_multi_uav_dataset/
- Multi-LiDAR GitHub: https://github.com/TIERS/multi_lidar_multi_uav_dataset
- Multi-LiDAR paper: https://arxiv.org/abs/2310.09165

## Cach noi trong bao cao/bao ve

- **Dataset:** "Em dung ODA lam benchmark chinh vi co MAV indoor, RGB/radar/IMU va OptiTrack ground truth."
- **An toan:** "Em khong chi hien thi video, ma bien ground truth thanh clearance/risk label/co so so sanh planner."
- **Planner:** "A* la grid-search baseline; RRT/RRT* la sampling-based planning; MPPI la sampling-based optimal control/optimization-based method."
- **Perception:** "Depth chi la relative depth feature, chua claim metric depth. Radar/IMU bo sung evidence da cam bien."
- **LiDAR:** "LiDAR hien duoc tach thanh nhanh stress-test tren ARCO/Multi-LiDAR: voxel/downsample, clustering va 3D bbox."
- **Tinh kha thi:** "Smoothness hien tai la proxy; dynamic feasibility day du cua UAV can them rang buoc velocity/acceleration/jerk/snap hoac MPC/trajectory optimization."

