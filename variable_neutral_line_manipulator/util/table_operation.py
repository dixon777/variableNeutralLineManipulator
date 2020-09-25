import numpy as np
def ensure_equal_num_cells_per_row(rows):
    max_row_len = max(len(r) for r in rows)
    for r in rows:
        r += [""]*(max_row_len - len(r))
    return rows

def ensure_equal_num_rows(tables):
    max_rows = max(len(t) for t in tables)
    for i in range(len(tables)):
        tables[i] += [[""]*len(tables[i][0]) for _ in range(max_rows - len(tables[i]))]
    return tables

def combine_tables(*tables):
    tables = [ensure_equal_num_cells_per_row(t) for t in tables]
    tables = ensure_equal_num_rows(tables)
    tables = [np.array(t) for t in tables]
    combined_table = tables[0]
    
    for t in tables[1:]:
        combined_table = np.hstack((combined_table, np.array([""]*len(t)).reshape((-1,1)), t))
    return combined_table