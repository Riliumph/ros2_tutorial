CREATE TABLE addresses (
  address_id serial NOT NULL PRIMARY KEY,
  student_id int NOT NULL,
  zip_code text NOT NULL,
  updated_at timestamptz DEFAULT CURRENT_TIMESTAMP,
  created_at timestamptz DEFAULT CURRENT_TIMESTAMP,
  FOREIGN KEY (student_id) REFERENCES students (student_id)
);
