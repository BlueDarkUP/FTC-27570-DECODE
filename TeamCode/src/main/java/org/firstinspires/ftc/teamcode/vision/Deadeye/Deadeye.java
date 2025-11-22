package org.firstinspires.ftc.teamcode.vision.Deadeye;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class Deadeye {

    // --- 原有参数 ---
    public static final double LL_VERTICAL_FOV_DEGREES = 49.7;
    private static final double CAMERA_TILT_DEGREES = 43;
    private static final double CAMERA_HEIGHT_CM = 23.5;
    private static final double ANCHOR_POINT_TY = -LL_VERTICAL_FOV_DEGREES / 2.0;
    private static final double INTAKE_ZONE_Y_ERROR_CM = 11.0;
    private static final double INTAKE_ZONE_X_ERROR_CM = 100.0;
    private static final double TRACKING_MATCHING_THRESHOLD_CM = 12.0;
    private static final double DELTA_TIME_SEC = 0.1;
    private static final int MAX_FRAMES_SINCE_SEEN = 3;
    private static final double SMOOTHING_FACTOR = 0.6;

    // --- 【新增】分类功能相关参数 ---
    /**
     * 定义分类区域的 Y 轴误差阈值 (cm)。像素的 y-error 小于此值时被认为进入了分类区。
     */
    private static final double CLASSIFICATION_ZONE_Y_ERROR_CM = 15.0;
    /**
     * 舵机对每个像素块执行分类动作的持续时间 (毫秒)。
     */
    private static final double CLASSIFICATION_DURATION_MS = 5200.0; // 2秒
    /**
     * 两次分类动作之间的暂停时间 (毫秒)，让舵机有时间复位或稳定。
     */
    private static final double PAUSE_DURATION_MS = 200.0; // 0.5秒

    // --- 硬件与状态变量 ---
    private final Limelight3A limelight;
    private final CRServo classifyMotor;
    private final ElapsedTime classificationTimer = new ElapsedTime();

    private List<LLResultTypes.DetectorResult> greenDetections = new ArrayList<>();
    private List<LLResultTypes.DetectorResult> purpleDetections = new ArrayList<>();
    private final double[] anchorPointPhysicalCoordinates;
    private int successfulIntakeCount = 0;
    private final List<TrackedTarget> activeTracks = new ArrayList<>();
    private static long nextTrackId = 0;

    // --- 【新增】分类功能的状态变量 ---
    /**
     * 分类任务的执行计划。列表中的每个元素代表一个要处理的像素块类型。
     */
    private List<PixelType> classificationPlan = new ArrayList<>();
    /**
     * 当前正在处理的分类计划中的像素索引。
     */
    private int classificationIndex = 0;
    /**
     * 分类过程的状态机。
     */
    private ClassificationState classificationState = ClassificationState.IDLE;

    /**
     * 【新增】用于表示像素类型的枚举，更清晰。
     */
    public enum PixelType {
        GREEN, PURPLE
    }

    /**
     * 【新增】分类过程的状态机枚举。
     * - IDLE: 空闲状态，等待新的分类任务。
     * - CLASSIFYING: 正在执行分类动作（舵机转动）。
     * - PAUSED: 两次分类动作之间的暂停状态。
     */
    private enum ClassificationState {
        IDLE, CLASSIFYING, PAUSED
    }

    private static class TrackedTarget {
        long id;
        double[] coordinates = new double[2];
        double[] velocity = new double[2];
        double[] alignmentError;
        int framesSinceSeen = 0;
        TrackedTarget(double[] coords, double[] error) { this.id = nextTrackId++; this.coordinates = coords; this.alignmentError = error; }
        void predict() { this.coordinates[0] += this.velocity[0] * DELTA_TIME_SEC; this.coordinates[1] += this.velocity[1] * DELTA_TIME_SEC; this.framesSinceSeen++; }
        void update(double[] measuredCoords, double[] newError) { double[] oldCoords = this.coordinates; this.coordinates[0] = SMOOTHING_FACTOR * measuredCoords[0] + (1 - SMOOTHING_FACTOR) * this.coordinates[0]; this.coordinates[1] = SMOOTHING_FACTOR * measuredCoords[1] + (1 - SMOOTHING_FACTOR) * this.coordinates[1]; this.velocity[0] = (this.coordinates[0] - oldCoords[0]) / DELTA_TIME_SEC; this.velocity[1] = (this.coordinates[1] - oldCoords[1]) / DELTA_TIME_SEC; this.alignmentError = newError; this.framesSinceSeen = 0; }
        boolean wasInIntakeZone() { if (alignmentError == null) return false; return Math.abs(alignmentError[0]) < INTAKE_ZONE_X_ERROR_CM && Math.abs(alignmentError[1]) < INTAKE_ZONE_Y_ERROR_CM; }
    }

    private static final Comparator<LLResultTypes.DetectorResult> distanceComparator =
            Comparator.comparingDouble(LLResultTypes.DetectorResult::getTargetYDegrees).reversed();

    public Deadeye(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        classifyMotor = hardwareMap.get(CRServo.class, "ClassifyServo");
        this.anchorPointPhysicalCoordinates = calculateCoordinatesFromAngles(0, ANCHOR_POINT_TY);
        limelight.pipelineSwitch(0);
    }

    public void start() { limelight.start(); }
    public void stop() { limelight.stop(); }

    public void update() {
        LLResult result = limelight.getLatestResult();
        greenDetections.clear();
        purpleDetections.clear();
        List<LLResultTypes.DetectorResult> allCurrentDetections = new ArrayList<>();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> allDetections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : allDetections) {
                if ("green".equals(detection.getClassName())) { greenDetections.add(detection); }
                else if ("purple".equals(detection.getClassName())) { purpleDetections.add(detection); }
            }
            greenDetections.sort(distanceComparator);
            purpleDetections.sort(distanceComparator);
            allCurrentDetections.addAll(greenDetections);
            allCurrentDetections.addAll(purpleDetections);
        }

        processIntakeCountV7(allCurrentDetections);

        processClassificationLogic(allCurrentDetections);
        updateClassificationMotor();
    }

    private void processClassificationLogic(List<LLResultTypes.DetectorResult> allDetections) {
        if (classificationState != ClassificationState.IDLE) {
            return;
        }

        List<LLResultTypes.DetectorResult> candidates = new ArrayList<>();
        for (LLResultTypes.DetectorResult detection : allDetections) {
            double[] error = calculateAlignmentError(detection);
            if (error != null && Math.abs(error[1]) < CLASSIFICATION_ZONE_Y_ERROR_CM) {
                candidates.add(detection);
            }
        }

        if (candidates.isEmpty()) {
            return;
        }

        candidates.sort(distanceComparator);

        this.classificationPlan = candidates.stream()
                .map(d -> "green".equals(d.getClassName()) ? PixelType.GREEN : PixelType.PURPLE)
                .collect(Collectors.toList());

        if (!this.classificationPlan.isEmpty()) {
            this.classificationState = ClassificationState.CLASSIFYING;
            this.classificationIndex = 0;
            this.classificationTimer.reset();
        }
    }

    private void updateClassificationMotor() {
        switch (classificationState) {
            case IDLE:
                classifyMotor.setPower(0);
                break;

            case CLASSIFYING:
                if (classificationTimer.milliseconds() >= CLASSIFICATION_DURATION_MS) {
                    classificationState = ClassificationState.PAUSED;
                    classificationTimer.reset();
                    classifyMotor.setPower(0);
                    break;
                }
                if (classificationIndex < classificationPlan.size()) {
                    PixelType currentTarget = classificationPlan.get(classificationIndex);
                    if (currentTarget == PixelType.GREEN) {
                        classifyMotor.setPower(-1.0);
                    } else {
                        classifyMotor.setPower(1.0);
                    }
                }
                break;

            case PAUSED:
                // 确保舵机停止
                classifyMotor.setPower(0);
                // 检查暂停时间是否结束
                if (classificationTimer.milliseconds() >= PAUSE_DURATION_MS) {
                    classificationIndex++; // 移动到下一个目标
                    // 检查是否所有目标都已处理完毕
                    if (classificationIndex >= classificationPlan.size()) {
                        // 序列完成，返回空闲状态
                        classificationState = ClassificationState.IDLE;
                        classificationPlan.clear();
                    } else {
                        // 还有目标，切换回分类状态
                        classificationState = ClassificationState.CLASSIFYING;
                        classificationTimer.reset();
                    }
                }
                break;
        }
    }

    /**
     * 【新增】检查系统当前是否正在执行分类序列。
     * @return 如果正在分类或暂停，返回 true；否则返回 false。
     */
    public boolean isClassifying() {
        return classificationState != ClassificationState.IDLE;
    }

    /**
     * 【新增】获取当前的分类计划，用于调试或显示。
     * @return 一个包含像素类型的列表，表示当前的动作序列。
     */
    public List<PixelType> getClassificationPlan() {
        return new ArrayList<>(classificationPlan);
    }


    // --- 原有方法 ---
    private void processIntakeCountV7(List<LLResultTypes.DetectorResult> currentDetections) {
        // ... 原有 V7 追踪算法代码保持不变 ...
        for (TrackedTarget track : activeTracks) { track.predict(); }
        Set<Integer> matchedDetectionIndices = new HashSet<>();
        List<TrackedTarget> tracksToMatch = new ArrayList<>(activeTracks);
        for (int i = 0; i < currentDetections.size(); i++) {
            LLResultTypes.DetectorResult detection = currentDetections.get(i);
            double[] coords = calculateCoordinates(detection); if (coords == null) continue;
            double bestDistance = Double.MAX_VALUE; TrackedTarget bestMatch = null;
            for (TrackedTarget track : tracksToMatch) {
                double dx = track.coordinates[0] - coords[0]; double dy = track.coordinates[1] - coords[1]; double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance < bestDistance) { bestDistance = distance; bestMatch = track; }
            }
            if (bestMatch != null && bestDistance < TRACKING_MATCHING_THRESHOLD_CM) {
                double[] error = calculateAlignmentError(detection); bestMatch.update(coords, error);
                tracksToMatch.remove(bestMatch); matchedDetectionIndices.add(i);
            }
        }
        Iterator<TrackedTarget> iterator = activeTracks.iterator();
        while (iterator.hasNext()) {
            TrackedTarget track = iterator.next();
            if (track.framesSinceSeen > MAX_FRAMES_SINCE_SEEN) {
                if (track.wasInIntakeZone()) { this.successfulIntakeCount++; }
                iterator.remove();
            }
        }
        for (int i = 0; i < currentDetections.size(); i++) {
            if (!matchedDetectionIndices.contains(i)) {
                LLResultTypes.DetectorResult newDetection = currentDetections.get(i);
                double[] newCoords = calculateCoordinates(newDetection); double[] newError = calculateAlignmentError(newDetection);
                if (newCoords != null && newError != null) { activeTracks.add(new TrackedTarget(newCoords, newError)); }
            }
        }
    }

    public int getSuccessfulIntakeCount() { return this.successfulIntakeCount; }

    public void resetIntakeCount() {
        this.successfulIntakeCount = 0;
        this.activeTracks.clear();
        nextTrackId = 0;
        // 【新增】同时重置分类状态
        this.classificationState = ClassificationState.IDLE;
        this.classificationPlan.clear();
        this.classifyMotor.setPower(0);
    }

    public double[] calculateAlignmentError(LLResultTypes.DetectorResult d) { if(d==null||anchorPointPhysicalCoordinates==null){return null;} double[] c=calculateCoordinates(d); if(c==null){return null;} return new double[]{c[0]-anchorPointPhysicalCoordinates[0], c[1]-anchorPointPhysicalCoordinates[1]}; }
    public double[] calculateCoordinates(LLResultTypes.DetectorResult d) { if(d==null){return null;} return calculateCoordinatesFromAngles(d.getTargetXDegrees(), d.getTargetYDegrees()); }
    private double[] calculateCoordinatesFromAngles(double tx, double ty) { double p=CAMERA_TILT_DEGREES-ty; if(p<=0||p>=90){return null;} double pr=Math.toRadians(p); double yr=Math.toRadians(tx); double y=CAMERA_HEIGHT_CM/Math.tan(pr); double x=y*Math.tan(yr); return new double[]{x, y}; }
    public List<LLResultTypes.DetectorResult> getGreenDetections() { return greenDetections; }
    public List<LLResultTypes.DetectorResult> getPurpleDetections() { return purpleDetections; }
    public LLResultTypes.DetectorResult getClosestGreen() { return greenDetections.isEmpty()?null:greenDetections.get(0); }
    public LLResultTypes.DetectorResult getClosestPurple() { return purpleDetections.isEmpty()?null:purpleDetections.get(0); }
    public LLResultTypes.DetectorResult getSecondClosestPurple() { return purpleDetections.size()<2?null:purpleDetections.get(1); }
    public LLResultTypes.DetectorResult getClosestTarget() { LLResultTypes.DetectorResult g=getClosestGreen(); LLResultTypes.DetectorResult p=getClosestPurple(); if(g==null&&p!=null){return p;} if(p==null&&g!=null){return g;} if(g==null){return null;} return g.getTargetYDegrees()>p.getTargetYDegrees()?g:p; }
    public LLStatus getStatus() { return limelight.getStatus(); }
    public LLResultTypes.DetectorResult getTargetClosestToAnchor() { List<LLResultTypes.DetectorResult> a=new ArrayList<>(); a.addAll(greenDetections); a.addAll(purpleDetections); if(a.isEmpty()||this.anchorPointPhysicalCoordinates==null){return null;} LLResultTypes.DetectorResult c=null; double m=Double.MAX_VALUE; for(LLResultTypes.DetectorResult d:a){ double[] t=calculateCoordinates(d); if(t!=null){ double ex=t[0]-this.anchorPointPhysicalCoordinates[0]; double ey=t[1]-this.anchorPointPhysicalCoordinates[1]; double s=ex*ex+ey*ey; if(s<m){m=s; c=d;}}} return c; }
}