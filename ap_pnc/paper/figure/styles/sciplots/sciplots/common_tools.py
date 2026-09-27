from pdf2image import convert_from_path
import os

def convert_pdf_to_images(pdf_path, output_dir, fmt='jpg', dpi=600):
    """
    Convert PDF to images (JPG or PNG)
    
    Args:
        pdf_path (str): Path to the PDF file
        output_dir (str): Directory to save the images
        fmt (str): Output format ('jpg' or 'png')
        dpi (int): DPI resolution for the output images
    """
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Get the filename without extension
    filename = os.path.splitext(os.path.basename(pdf_path))[0]
    
    try:
        # Convert PDF to images
        pages = convert_from_path(pdf_path, dpi=dpi)
        
        # Save each page as an image
        for i, page in enumerate(pages):
            image_path = os.path.join(output_dir, f"{filename}_page_{i+1}.{fmt}")
            if fmt.lower() == 'jpg':
                page.save(image_path, 'JPEG')
            else:
                page.save(image_path, 'PNG')
            print(f"Saved: {image_path}")
            
    except Exception as e:
        print(f"Error converting PDF: {str(e)}")

# Example usage
if __name__ == "__main__":
    # Example PDF file path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(current_dir)
    
    figure_dir = os.path.join(root_dir, "examples", "figures")
    
    # Output directory for images
    input_image_pdf = os.path.join(figure_dir, "figure-aero-lyu.pdf")
    
    # Convert PDF to JPG
    convert_pdf_to_images(input_image_pdf, figure_dir, fmt='jpg')
    
    # Or convert to PNG
    convert_pdf_to_images(input_image_pdf, figure_dir, fmt='png')