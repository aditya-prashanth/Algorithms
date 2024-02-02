In creating this project, here is a preliminary outline of the components of the search algorithm software:
Master:
- A master program between all USVs that takes in a search area and assigns a search path to each utilized USV
- Estimates area of untraversable terrains and marks it off of the area needing to be searched
- Pinpoints the location of the target and sends a signal to search teams so they can navigate towards it
- Has a live update map that displays exactly which areas have been searched and not for search teams to stay informed

Individual:
- Marks traversed areas and sends a signal back to master than an area has been searched
- Marks untraversable terrain as such so other USVs don't attempt to search it
- Marks the target and sends a signal up to master
- Has some sort of reaction mechanism upon finding the victim to protect it
