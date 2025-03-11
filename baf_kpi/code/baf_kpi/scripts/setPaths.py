from pathlib import Path

def setPaths(experiment_dir, subject_id, trial_id, opts=None):
    dataset_root = Path(experiment_dir).resolve()
    path_to_subject = dataset_root / f'S{subject_id:02d}'
    path_to_trial = path_to_subject / f'trial{trial_id:02d}'
    path_to_raw_data = path_to_trial / 'raw_dataplayer'
    path_to_processed_data = path_to_trial / 'processed'
    path_to_plots = path_to_trial / 'plots'
    path_to_videos = path_to_trial / 'videos'

    paths = {
        "datasetRoot": dataset_root.as_posix(),
        "pathToSubject": path_to_subject.as_posix(),
        "pathToTrial": path_to_trial.as_posix(),
        "pathToRawData": path_to_raw_data.as_posix(),
        "pathToProcessedData": path_to_processed_data.as_posix(),
        "pathToPlots": path_to_plots.as_posix(),
        "pathToVideos": path_to_videos.as_posix()
    }

    return paths