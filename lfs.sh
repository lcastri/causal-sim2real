#!/bin/bash

# --- CONFIGURATION ---
# List of all branches that need LFS history purged.
# DO NOT include 'sim-to-real' since you already fixed it.
BRANCHES=("main" "dev" "dev-task" "robot")

# Files to be converted from LFS pointers to regular files.
FILE1="HRISim_docker/src/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl"
FILE2="HRISim_docker/src/HRISim/prediction/hrisim_prediction_manager/DAGs/res.pkl"

# --- EXECUTION ---

echo "Starting Git LFS Cleanup Script..."
echo "Files to process: $FILE1, $FILE2"
echo "Branches to process: ${BRANCHES[*]}"
echo "-------------------------------------"

for branch in "${BRANCHES[@]}"
do
    echo "Processing branch: $branch"

    # 1. Checkout the branch
    git checkout "$branch" || { echo "ERROR: Could not checkout branch $branch. Exiting."; exit 1; }

    # 2. Local LFS Cleanup (ensures local state is clean for filter-repo)
    git lfs untrack "$FILE1"
    git lfs untrack "$FILE2"
    git lfs uninstall --local # Remove local LFS hooks

    # Clear .gitattributes file to remove any LFS patterns
    echo "" > .gitattributes
    git add .gitattributes
    
    # 3. Stage and Commit the final clean configuration
    # We remove from index, re-add, and commit to ensure files are tracked regularly
    git rm --cached "$FILE1" "$FILE2" 2>/dev/null
    git add "$FILE1" "$FILE2"
    git commit -m "Cleanup: Convert LFS files to regular Git objects." || echo "No new commit needed on $branch, proceeding to history rewrite."

    # 4. Rewrite History (The destructive part)
    # This command converts all past LFS pointers for small files into actual file content.
    echo "Rewriting history to permanently remove LFS pointers..."
    git filter-repo --strip-blobs-bigger-than 10M --force || { echo "ERROR: filter-repo failed on $branch. Aborting branch. Proceeding to next branch (if available)."; continue; }
    
    # 5. Force Push the Rewritten History
    echo "Force pushing clean history to remote 'origin'..."
    # NOTE: You will be prompted for your GitHub PAT for each push.
    git push --force origin "$branch" || { echo "ERROR: Force push failed on $branch. Please check credentials/permissions."; continue; }

    echo "Successfully cleaned and pushed branch: $branch"
    echo "-------------------------------------"

done

echo "Script finished. All selected branches processed."