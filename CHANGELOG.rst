^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package surface_path_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-22)
------------------
* compute_joint_path: solve the task-redundant IK in parallel. New parameters ik_parallel
  (default true) and ik_threads (default 0 = all the ik_solver node's threads) set the parallelize
  and num_threads fields of the get_task_redundant_ik_array request, overriding parallel_ik_mode
  of the ik_solver node. Set ik_parallel to false to restore the previous sequential behaviour.
* Require ik_solver_msgs >= 0.1.0 (num_threads field of GetIkArray).
* Contributors: Manuel Beschi
